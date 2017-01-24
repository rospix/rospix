#include <ros/ros.h>
#include <ros/package.h>

#include "timepix_handler.h"
#include <string>

// timepix driver headers
#include "usb.h"

// for reading a file
#include <iostream>
#include <fstream>

#include <thread>
#include <mutex>

using namespace std;

// the constructor
TimepixHandler::TimepixHandler(ros::NodeHandle nh, string idname, string equalization_directory) {

  this->equalization_directory = equalization_directory;

  opened = false;
  equalization_loaded = false;
  exposing = false;
  dummy = false;

  memset(equalization, 0, MATRIX_SIZE * sizeof(uint8_t));

  nh_ = nh;
  idname_ = idname;

  nh_.param("name", name_, string());

  if (name_.compare(string("dummy")) == 0) {

    dummy = true;
    opened = true;

    nh_.param("simulate_focus", dummy_simulate_focus, false);
    nh_.param("photon_flux", dummy_photon_flux, 100);
    nh_.param("simulate_background", simulate_background, false);
    nh_.param("n_images", dummy_n_images, 0);
    nh_.param("optics_dimension", optics_dimension, 1);

  } else {

    // load defaults from the config

    int tempint = 0;
    nh_.param("defaults/threshold", tempint, 0);
    threshold = uint16_t(tempint);

    if (threshold == 0) {
      ROS_ERROR("%s: Error loading the default threshold from config file.", idname_.c_str());
    }

    nh_.param("defaults/bias", bias, 0.0);

    if (bias == 0) {
      ROS_ERROR("%s: Error loading the default bias from the config file.", idname_.c_str());
    }

    nh_.param("equalization", equalization_file, string());

    if (equalization_file.empty()) {

      ROS_ERROR("%s: Error loading the name of the equalization file from the config file.", idname_.c_str());
    }

    // load dacs
    std::vector<double> tempList;
    int tempIdx = 0;

    tempIdx = 0;
    nh_.getParam("defaults/dacs", tempList);

    if (tempList.size() != 14) {
      ROS_ERROR("%s: DAC array in the config have wrong length.", idname_.c_str());
    } else {
      for (int i = 0; i < 14; i++) {
        dacs[i] = tempList[tempIdx++];
      }
    }

  }

  nh_.param("defaults/exposure", exposure, 0.0);

  if (exposure == 0.0) {
    ROS_ERROR("%s: Error loading the default exposure time from the config file.", idname_.c_str());
  }

  memset(image, 0, MATRIX_SIZE * sizeof(uint16_t));

  ROS_INFO("%s: Initializing sensor \"%s\".", idname_.c_str(), name_.c_str());
}

void TimepixHandler::changeState(State_t new_state) {

  switch (new_state) {

    case IDLE:

      exposing = false;

      break;

    default:

      if (!opened) {

        ROS_ERROR_THROTTLE(1, "%s: Cannot execute measurement, device not opened.", idname_.c_str());

        ROS_INFO("%s: Trying to reopen the device \"%s\"", idname_.c_str(), name_.c_str());

        if (!reOpen()) {

          return;
        }
      }

      break;
  }

  current_state = new_state;
}

void TimepixHandler::doSingleExposure(void) {

  // save the settings to the output image before taking the exposure...
  outputImage.stamp = ros::Time::now();
  outputImage.interface = name_;
  outputImage.threshold = threshold;
  outputImage.bias = bias;
  outputImage.mode = mode;
  outputImage.exposure_time = exposure;

  if (!loadDacs()) {

    ROS_ERROR("%s: Error during setting DACs before an exposure", idname_.c_str());
  }

  if (!setNewBias(bias)) {

    ROS_ERROR("%s: Error during setting bias before an exposure", idname_.c_str());
  }

  if (!doExposure(exposure)) {

    ROS_ERROR("%s: Error while doing single exposure, \"%s\".", idname_.c_str(), name_.c_str());

  } else {

    if (!readImage()) {

      ROS_ERROR("%s: Could not read the image from \"%s\".", idname_.c_str(), name_.c_str());

    } else {

      publishImage();
    }
  }
}

void TimepixHandler::mainThread(void) {

  ros::Duration d(0.1);

  while (ros::ok) {

    // for computing time utilization
    exposure_started = ros::Time::now();
    total_exposing_time = 0;

    switch (current_state) {

      case IDLE:

        d.sleep();

        break;

      case SINGLE_EXPOSURE:

        doSingleExposure();

        ROS_INFO("%s: exposure finished, time utilization %1.2f%%", idname_.c_str(), 100.0*(total_exposing_time/(ros::Time::now()-exposure_started).toSec()));

        changeState(IDLE);

        break;


        break;

      case CONTINOUS_EXPOSURE:

        exposing = true;

        ROS_INFO("%s: starting continuous exposure, exposure time %3.3f s", idname_.c_str(), exposure);

        while (ros::ok && exposing && opened) {

          doSingleExposure();
          ROS_INFO_THROTTLE(5, "%s: exposures in progress, time utilization %1.2f%%", idname_.c_str(), 100*(total_exposing_time/(ros::Time::now()-exposure_started).toSec()));
        }

        changeState(IDLE);

        break;

      case BATCH_EXPOSURE:

        exposing = true;

        ROS_INFO("%s: starting batch exposure, exposure time %3.3f, %d exposures", idname_.c_str(), exposure, batch_exposure_count);

        for (int i = 0; i < batch_exposure_count; i++) {

          if (!opened || !exposing) {
            break;            
          }

          doSingleExposure();
        }

        ROS_INFO("%s: batch exposure finished, time utilization %1.2f%%", idname_.c_str(), 100*(total_exposing_time/(ros::Time::now()-exposure_started).toSec()));

        changeState(IDLE);

        break;
    }
  }
}

bool TimepixHandler::compareStrings(const char * a, const char * b) {

  for (int i = 0; i < 50; i++) {

    if (a[i] != b[i]) {
      return false;
    }

    if (a[i] == '\0' && b[i] == '\0') {
      return true;
    }
  }
}

bool TimepixHandler::open(void) {

  const char * devNames[50];
  int devCount = 0;

  if (dummy) {

    opened = true;

    ROS_INFO("%s: Successfully opened device \"%s\".", idname_.c_str(), name_.c_str());

  } else {

    int error = 1;

    listDevices(devNames, &devCount);

    // find the device
    for (int i = 0; i < devCount; i++) {
      if (compareStrings(devNames[i], name_.c_str())) {
        error = openDevice(name_.c_str(), &id);
        break;
      }
    }

    // success?
    if (error == 0) {

      chip_id = chipID(id);

      ROS_INFO("%s: Successfully opened USB Lite \"%s\", its chip is is \"%s\".", idname_.c_str(), name_.c_str(), chip_id.c_str());

      interface = USB_LITE;  
      opened = true;

    } else {

      error = 1;

      listDevicesFpx(devNames, &devCount);
      // find the device
      for (int i = 0; i < devCount; i++) {
        if (compareStrings(devNames[i], name_.c_str())) {
          error = openDeviceFpx(name_.c_str(), &id);
          break;
        }
      }

      // success?
      if (error == 0) {

        chip_id = chipID(id);

        ROS_INFO("%s: Successfully opened FitPix \"%s\", its chip is is \"%s\".", idname_.c_str(), name_.c_str(), chip_id.c_str());

        interface = FITPIX;
        opened = true;
      }
    }
  }

  if (opened) {

    // advertise services
    service_single_exposure = nh_.advertiseService("do_single_exposure", &TimepixHandler::singleExposureCallback, this);
    service_continuous_exposure = nh_.advertiseService("do_continuous_exposure", &TimepixHandler::continuouExposureCallback, this);
    service_batch_exposure = nh_.advertiseService("do_batch_exposure", &TimepixHandler::batchExposureCallback, this);
    service_set_mode = nh_.advertiseService("set_mode", &TimepixHandler::setModeCallback, this);
    service_set_bias = nh_.advertiseService("set_bias", &TimepixHandler::setBiasCallback, this);
    service_set_threshold = nh_.advertiseService("set_threshold", &TimepixHandler::setThresholdCallback, this);
    service_set_exposure = nh_.advertiseService("set_exposure_time", &TimepixHandler::setExposureCallback, this);
    service_interrupt_measurement = nh_.advertiseService("interrupt_measurement", &TimepixHandler::interruptMeasurementCallback, this);

    // create publishers
    image_publisher = nh_.advertise<rospix::Image>("image", 1);

    if (!loadDacs()) {
      ROS_ERROR("%s: Error while loading DACs after openning the device.", idname_.c_str());
    }

    if (!setNewBias(bias)) {
      ROS_ERROR("%s: Error while setting bias after openning the device.", idname_.c_str());
    }

    if (!loadEqualization()) {

      ROS_ERROR("%s: Failed to load the equalization matrix.", idname_.c_str());  
      return false; 
    }

    if (!setEqualization()) {

      ROS_ERROR("%s: Failed to set the equalization matrix, probably communication problem.", idname_.c_str());
      return false;
    }

    int tempmode = 0;
    nh_.param("defaults/mode", tempmode, 0);

    if (!setMode(tempmode)) {

      ROS_ERROR("%s: Failed to set sensor mode.", idname_.c_str());
      return false;
    }

    // start the thread
    changeState(IDLE);
    main_thread = std::thread(&TimepixHandler::mainThread, this);
  }

  return opened;
}

bool TimepixHandler::reOpen(void) {

  if (dummy)
    return true;

  const char * devNames[50];
  int devCount = 0;

  int error = 1;

  if (interface == USB_LITE) {

    listDevices(devNames, &devCount);

    // find the device
    for (int i = 0; i < devCount; i++) {
      if (compareStrings(devNames[i], name_.c_str())) {
        error = openDevice(name_.c_str(), &id);
        break;
      }
    }

    // success?
    if (error == 0) {

      if (!loadEqualization() || !setNewBias(bias)) {

        return false;
      }

      ROS_INFO("%s: Successfully reopened USB Lite \"%s\".", idname_.c_str(), name_.c_str());

      opened = true;
    }
  } else if (interface == FITPIX) {

    listDevicesFpx(devNames, &devCount);
    // find the device
    for (int i = 0; i < devCount; i++) {
      if (compareStrings(devNames[i], name_.c_str())) {
        error = openDeviceFpx(name_.c_str(), &id);
        break;
      }
    }

    // success?
    if (error == 0) {

      if (!loadEqualization() || !setNewBias(bias)) {

        return false;
      }

      ROS_INFO("%s: Successfully reopened FitPix \"%s\".", idname_.c_str(), name_.c_str());

      opened = true;
    }
  }

  if (!opened) {

    ROS_ERROR("%s: Reopenning of FitPix \"%s\" failed.", idname_.c_str(), name_.c_str());
  }

  return opened;
}

bool TimepixHandler::singleExposureCallback(rospix::Exposure::Request &req, rospix::Exposure::Response &res) {

  if (req.exposure_time <= 0) {

    res.success = false;
    res.message = "The exposure time should be positive.";
    return true;
  }

  if (current_state != IDLE) {

    res.message = "Measurement in progress.";
    res.success = false;
    return true;
  }

  exposure = req.exposure_time;

  changeState(SINGLE_EXPOSURE);

  ros::Duration d(0.001);
  while (ros::ok && current_state != IDLE) {

    d.sleep();
  }

  res.success = true;
  res.message = "Meaurement finihed.";
  return true;
}

bool TimepixHandler::loadDacs(void) {

  if (dummy)
    return true;

  dacs[6] = threshold;

  int rc = 0;

  switch (interface) {

    case USB_LITE:

      rc = setDacs(id, dacs);
      break;

    case FITPIX:

      rc = setDacsFpx(id, dacs);
      break;
  }

  if (rc == 0) {

    return true;
  } else {

    opened = false;
    return false;
  }
}

bool TimepixHandler::setThreshold(const uint16_t newThreshold) {

  if (dummy)
    return true;

  dacs[6] = newThreshold;

  return loadDacs();
}

bool TimepixHandler::setNewBias(const double newBias) {

  if (dummy)
    return true;

  int rc = 0;

  switch (interface) {

    case USB_LITE:

      rc = setBias(id, newBias);      

      break;

    case FITPIX:

      rc = setBiasFpx(id, newBias);

      break;
  }

  if (rc == 0) {
    return true;
  } else {
    return false;
  }
}

bool TimepixHandler::loadEqualization(void) {

  if (dummy)
    return true;

  string path = equalization_directory + equalization_file;

  FILE * F;
  F = fopen(path.c_str(), "rb");

  if (F == NULL) {

    ROS_ERROR("%s: Cannot read the equalization from %s!", idname_.c_str(), path.c_str());
    return false;
  }

  uint8_t tempByte;
  for (int i = 0; i < 65536; i++) {

    if (fread(&tempByte, 1, 1, F) != 1) {

      ROS_ERROR("%s: Error reading eqialization file!", idname_.c_str());
      return false;
    }

    equalization[i] = tempByte;
  }

  equalization_loaded = true;

  return true;
}

bool TimepixHandler::setEqualization(void) {

  if (dummy)
    return true;

  int rc = 0;

  if (!equalization_loaded) {

    ROS_ERROR("%s: Cannot set the equalization, matrix not loaded.", idname_.c_str());
    return false;
  }

  switch (interface) {

    case USB_LITE:

      rc = writePixCfg(id, equalization, MATRIX_SIZE);

      break;

    case FITPIX:

      rc = writePixCfgFpx(id, equalization, MATRIX_SIZE);

      break;
  }

  if (!readImage()) {

    ROS_WARN("%s: Error while cleaning sensor after loading equalization.", idname_.c_str());
  }

  memset(image, 0, MATRIX_SIZE * sizeof(uint16_t));

  if (rc == 0) {
    return true;
  } else {
    opened = false;
    return false;
  }
}

bool TimepixHandler::doExposure(double time) {

  int rc = 0;

  ros::Time startTime = ros::Time::now();

  if (dummy) {

    // simulate the exposure time
    simulateExposure();

  } else {

    switch (interface) {

      case USB_LITE:

        rc = doAcquisition(id, time);

        break;

      case FITPIX:

        rc = doAcquisitionFpx(id, time);

        break;
    }
  }

  total_exposing_time += (ros::Time::now() - startTime).toSec();

  if (rc == 0) {

    return true;

  } else {

    opened = false;
    return false;

  }
}

double TimepixHandler::samplePseudoNormal(double mean, double std) {

  double temp = 0;

  for (int i = 0; i < 10; i++) {

    temp += randf(-1, 1) * std;
  }

  temp /= 10.0;

  temp += mean;

  return temp;
}

double TimepixHandler::randf(double from, double to) {

  double zero_to_one = double((float) rand()) / double(RAND_MAX);

  return (to - from)*zero_to_one + from;
}

int TimepixHandler::randi(int from, int to) {

  double zero_to_one = double((float) rand()) / double(RAND_MAX);

  return int(floor(to - from)*zero_to_one) + from;
}

void TimepixHandler::simulateExposure(void) {

  ros::Time start_time = ros::Time::now();

  // load some random background noise
  memset(image, 0, MATRIX_SIZE * sizeof(uint16_t));

  // simulate the photons
  if (dummy_simulate_focus) {

    int x, y;
    for (int i = 0; i < int(dummy_photon_flux*exposure); i++) {

      y = int(samplePseudoNormal(128, 64));

      if (y < 0 || y > 255)
        continue;

      if (optics_dimension == 1)
        x = floor(randi(0, 256));
      else {

        x = int(samplePseudoNormal(128, 64));

        if (x < 0 || y > 255)
          continue;

      }

      image[y*256 + x] += randi(1, 250);
    }

  } else {

    for (int i = 0; i < int(dummy_photon_flux*exposure); i++) {

      image[randi(0, 65536)] += randi(1, 250);
    } 
  }

  // simulate background
  if (simulate_background) {

    // load nth dummy image
    char temp[30];
    sprintf(temp, "/dummy/%d.txt", dummy_counter);

    if (++dummy_counter > (dummy_n_images-1)) {
      dummy_counter = 0;
    }

    string path = ros::package::getPath("rospix")+string(temp);

    FILE * f = fopen(path.c_str(), "r");

    int tempint;

    if (f != NULL) {

      for (int i = 0; i < 256; i++) {

        for (int j = 0; j < 256; j++) {

          fscanf(f, "%d ", &tempint);
          image[j + i*256] += tempint;
        }

        fscanf(f, "\n");
      }
      fclose(f);
    } else {
      ROS_WARN("%s: Cannot open file with dummy radiation background", idname_.c_str());
    }
  }

  double dt = (ros::Time::now() - start_time).toSec();

  if (dt < exposure) {
    ros::Duration d(exposure - dt);
    d.sleep();
  }
}

bool TimepixHandler::readImage(void) {

  if (dummy) {

    return true;
  }

  int rc = 0;

  switch (interface) {

    case USB_LITE:

      rc = readMatrix(id, image, MATRIX_SIZE, true);

      break;

    case FITPIX:

      rc = readMatrixFpx(id, image, MATRIX_SIZE, true);

      break;
  }

  if (rc == 0) {
    return true;
  } else {
    opened = false;
    return false;
  }
}

bool TimepixHandler::publishImage(void) {

  for (int i = 0; i < 65536; i++) {
    outputImage.image[i] = image[i];
  }

  image_publisher.publish(outputImage);
}

bool TimepixHandler::setMode(int newmode) {

  if (mode == newmode) {

    return true;
  }

  ROS_INFO("%s: Setting mode to %d", idname_.c_str(), newmode);

  for (int i = 0; i < MATRIX_SIZE; i++) {
    if (newmode == 0)
      equalization[i] = (equalization[i] & 0b00111111);
    else
      equalization[i] = (equalization[i] | 0b01000000);
  }

  if (!setEqualization()) {

    ROS_ERROR("%s: Error during loading equalization while setting a mode.", idname_.c_str());
    return false;

  } else {

    mode = newmode;
    return true;
  }
}

bool TimepixHandler::setModeCallback(rospix::SetInt::Request &req, rospix::SetInt::Response &res) {

  if (opened) {

    if (current_state == IDLE) {

      if (!setMode(req.value)) {

        res.message = "Error while setting mode.";
        res.success = false;

      } else {

        res.success = true;
        char tempChar[20];
        sprintf(tempChar, "Mode changed to %s.", ((mode == MPX) ? "MPX" : "TOT"));
        res.message = string(tempChar);
      }

    } else {

      res.message = "Cannot set the mode during measurement.";
      res.success = false;
    }

  } else {

    res.message = "Device not openned!";
    res.success = false;
  }

  return true;
}

bool TimepixHandler::continuouExposureCallback(rospix::Exposure::Request &req, rospix::Exposure::Response &res) {

  if (req.exposure_time <= 0) {

    res.success = false;
    res.message = "The exposure time should be positive.";
    return true;
  }

  if (current_state != IDLE) {

    if (current_state == CONTINOUS_EXPOSURE) {

      exposure = req.exposure_time;
      res.message = "Already measuring, changing the exposure time.";
      return true;
    }

    res.message = "Measurement in progress.";
    res.success = false;
    return true;
  }

  exposure = req.exposure_time;
  total_exposing_time = 0;
  exposure_started = ros::Time::now();

  changeState(CONTINOUS_EXPOSURE);

  res.success = true;
  res.message = "Meaurement started.";
  return true;
}

bool TimepixHandler::batchExposureCallback(rospix::BatchExposure::Request &req, rospix::BatchExposure::Response &res) {

  if (req.exposure_time <= 0) {

    res.success = false;
    res.message = "The exposure time should be positive.";
    return true;
  }

  if (req.exposure_count < 1) {

    res.success = false;
    res.message = "The exposure count should be positive.";
    return true;
  }

  if (current_state == IDLE) {

    batch_exposure_count = req.exposure_count;
    exposure = req.exposure_time;

    changeState(BATCH_EXPOSURE);

    res.success = true;
    res.message = "Meaurement started.";
    return true;
  }
}

bool TimepixHandler::interruptMeasurementCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (exposing) {

    exposing = false;

    ros::Duration d(0.01);
    while (ros::ok && current_state != IDLE) {

      d.sleep();
    }

    res.success = true;
    res.message = "Measurement interrupted";
  } else {

    res.success = true;
    res.message = "Measurement already finished";
  }

  return true;
}

bool TimepixHandler::setThresholdCallback(rospix::SetInt::Request &req, rospix::SetInt::Response &res) {

  if (req.value >= 0 && req.value <= 1000) {

    threshold = req.value;
    res.success = true;
    res.message = "Threshold changed.";
    return true;

  } else {

    res.success = false;
    res.message = "Threshold is out of bounds [0, 1000]";
    return true;
  }
}

bool TimepixHandler::setBiasCallback(rospix::SetDouble::Request &req, rospix::SetDouble::Response &res) {

  if (req.value < 5 || req.value > 94) {

    res.success = false;
    res.message = "Bias voltage out of bounds [5.4, 94] V.";
    return true;
  }

  bias = req.value;

  res.success = true;
  res.message = "New bias set.";
  return true;
}

bool TimepixHandler::setExposureCallback(rospix::SetDouble::Request &req, rospix::SetDouble::Response &res) {

  if (req.value <= 0) {

    res.success = false;
    res.message = "Exposure time should be positive.";
    return true;
  }

  exposure = req.value;

  res.success = true;
  res.message = "New exposure time set.";
  return true;
}
