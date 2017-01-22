#include <ros/ros.h>

#include "timepix_handler.h"
#include <string>

// timepix driver headers
#include "usb.h"

// for reading a file
#include <iostream>
#include <fstream>

#include <thread>
#include <mutex>

#include <image_transport/image_transport.h>

using namespace std;

// the constructor
TimepixHandler::TimepixHandler(ros::NodeHandle nh) {

  opened = false;
  equalization_loaded = false;
  exposing = false;
  dummy = false;
  memset(equalization, 0, MATRIX_SIZE * sizeof(uint8_t));

  this->nh_ = nh;

  nh_.param("name", name_, string());

  if (name_.compare(string("dummy")) == 0) {

    dummy = true;
    opened = true;

    nh_.param("simulate_focus", dummy_simulate_focus, false);
    nh_.param("photon_flux", dummy_photon_flux, 100);

  } else {

    // load defaults from the config

    int tempint = 0;
    nh_.param("defaults/threshold", tempint, 0);
    threshold = uint16_t(tempint);

    if (threshold == 0) {
      ROS_ERROR("Error loading the default threshold from config file.");
    }

    nh_.param("defaults/bias", bias, 0.0);

    if (bias == 0) {
      ROS_ERROR("Error loading the default bias from the config file.");
    }

    nh_.param("equalization", equalization_file, string());

    if (equalization_file.empty()) {
      ROS_ERROR("Error loading the name of the equalization file from the config file.");
    }

    // load dacs
    std::vector<double> tempList;
    int tempIdx = 0;

    tempIdx = 0;
    nh_.getParam("defaults/dacs", tempList);

    if (tempList.size() != 14) {
      ROS_ERROR("DAC array in the config have wrong length.");
    } else {
      for (int i = 0; i < 14; i++) {
        dacs[i] = tempList[tempIdx++];
      }
    }

  }

  nh_.param("defaults/exposure", exposure, 0.0);

  if (exposure == 0.0) {
    ROS_ERROR("Error loading the default exposure time from the config file.");
  }

  memset(image, 0, MATRIX_SIZE * sizeof(uint16_t));

  ROS_INFO("Initializing sensor \"%s\".", name_.c_str());
}

void TimepixHandler::changeState(State_t new_state) {

  current_state = new_state;

  switch (new_state) {

    case IDLE:

      exposing = false;

      break;
  }
}

void TimepixHandler::doSingleExposure(void) {

  if (!loadDacs()) {

    ROS_ERROR("Error during setting DACs before an exposure");
  }

  if (!doExposure(exposure)) {

    ROS_ERROR("Error while doing single exposure.");

  } else {

    if (!readImage()) {

      ROS_ERROR("Could not read the image.");

    } else {

      publishImage();
    }
  }
}

void TimepixHandler::mainThread(void) {

  ros::Duration d(0.1);

  while (ros::ok) {

    switch (current_state) {

      case IDLE:

        d.sleep();

        break;

      case SINGLE_EXPOSURE:

        ROS_INFO("THREAD: started single exposure job");

        doSingleExposure();

        changeState(IDLE);

        break;

      case CONTINOUS_EXPOSURE:

        ROS_INFO("THREAD: started continuous exposure job");

        exposing = true;

        while (ros::ok && exposing) {

          doSingleExposure();
        }

        changeState(IDLE);

        break;

      case BATCH_EXPOSURE:

        ROS_INFO("THREAD: started exposure batch job");

        exposing = true;

        for (int i = 0; i < batch_exposure_count; i++) {

          doSingleExposure();
        }

        changeState(IDLE);

        break;
    }
  }
}

bool TimepixHandler::open(void) {

  int error = 0;

  // list the devices, this is neccessary to open them
  const char* devNames[50];
  int devCount = 0;
  listDevices(devNames, &devCount);

  // try to open usb lite interface
  error = openDevice(name_.c_str(), &id);

  // success?
  if (error == 0) {

    chip_id = chipID(id);

    ROS_INFO("Successfully opened USB Lite \"%s\", its chip is is \"%s\".", name_.c_str(), chip_id.c_str());

    interface = USB_LITE;  
    opened = true;

  } else {

    // try to open fitpix device
    listDevicesFpx(devNames, &devCount);

    error = openDeviceFpx(name_.c_str(), &id);

    // success?
    if (error == 0) {

      chip_id = chipID(id);

      ROS_INFO("Successfully opened FitPix \"%s\", its chip is is \"%s\".", name_.c_str(), chip_id.c_str());

      interface = FITPIX;
      opened = true;
    }
  }

  if (dummy) {

    opened = true;

    ROS_INFO("Successfully opened device \"%s\".", name_.c_str());
  }

  if (opened) {

    // advertise services
    service_single_exposure = nh_.advertiseService("do_single_exposure", &TimepixHandler::singleExposureCallback, this);
    service_continuous_exposure = nh_.advertiseService("do_continuous_exposure", &TimepixHandler::continuouExposureCallback, this);
    service_batch_exposure = nh_.advertiseService("do_batch_exposure", &TimepixHandler::batchExposureCallback, this);
    service_set_mode = nh_.advertiseService("set_mode", &TimepixHandler::setModeCallback, this);
    service_set_bias = nh_.advertiseService("set_bias", &TimepixHandler::setBiasCallback, this);
    service_set_threshold = nh_.advertiseService("set_threshold", &TimepixHandler::setThresholdCallback, this);
    service_interrupt_measurement = nh_.advertiseService("interrupt_measurement", &TimepixHandler::interruptMeasurementCallback, this);

    // create publishers
    image_transport::ImageTransport it(nh_);
    image_publisher = it.advertise("image", 1);

    if (!loadDacs()) {
      ROS_ERROR("Error while loading DACs after openning the device.");
    }

    if (!setNewBias(bias)) {
      ROS_ERROR("Error while setting bias after openning the device.");
    }

    if (!loadEqualization(equalization_file)) {

      ROS_ERROR("Failed to load the equalization matrix.");  
      return false; 
    }

    if (!setEqualization()) {

      ROS_ERROR("Failed to set the equalization matrix, probably communication problem.");
      return false;
    }

    int tempmode = 0;
    nh_.param("defaults/mode", tempmode, 0);

    if (!setMode(tempmode)) {

      ROS_ERROR("Failed to set sensor mode.");
      return false;
    }

    // start the thread
    changeState(IDLE);
    main_thread = std::thread(&TimepixHandler::mainThread, this);
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

    res.success = false;
    res.message = "We are already measuring.";
    return true;
  }

  exposure = req.exposure_time;

  current_state = SINGLE_EXPOSURE;

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

bool TimepixHandler::loadEqualization(const string filename) {

  if (dummy)
    return true;

  FILE *F;
  F=fopen(filename.c_str(), "rb");
  if (!F){
    ROS_ERROR("Cannot read the equalization!");
    return false;
  }

  uint8_t tempByte;
  for (int i = 0; i < 65536; i++) {

    if (fread(&tempByte, 1, 1, F) != 1) {

      ROS_ERROR("Error reading eqialization file!");
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

    ROS_ERROR("Cannot set the equalization, matrix not loaded.");
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

    ROS_WARN("Error while cleaning sensor after loading equalization.");
  }

  memset(image, 0, MATRIX_SIZE * sizeof(uint16_t));

  if (rc == 0) {
    return true;
  } else {
    return false;
  }
}

bool TimepixHandler::doExposure(double time) {

  if (dummy) {

    // simulate the exposure time
    ros::Duration d(exposure);
    d.sleep();
    return true;
  }

  int rc = 0;

  switch (interface) {

    case USB_LITE:

      rc = doAcquisition(id, time);

      break;

    case FITPIX:

      rc = doAcquisitionFpx(id, time);

      break;
  }

  if (rc == 0) {
    return true;
  } else {
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

  // load some random background noise
  memset(image, 0, MATRIX_SIZE * sizeof(uint16_t));

  // simulate the photons
  if (dummy_simulate_focus) {

    int x, y;
    for (int i = 0; i < int(dummy_photon_flux*exposure); i++) {

      y = int(samplePseudoNormal(128, 64));

      if (y < 0 || y > 255)
        continue;

      x = floor(randi(0, 256));

      image[y*256 + x] = randi(1, 11);
    }

  } else {

    for (int i = 0; i < int(dummy_photon_flux*exposure); i++) {

      image[randi(0, 65536)] = randi(1, 11);
    } 
  }
}

bool TimepixHandler::readImage(void) {

  if (dummy) {

    simulateExposure();
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
    return false;
  }
}

bool TimepixHandler::publishImage(void) {

  cv::Mat cv_image = cv::Mat::zeros(256, 256, CV_16UC1);

  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      cv_image.at<uint16_t>(i, j) = image[j*256 + i] > 0 ? 65535 : 0; 
    }
  }

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", cv_image).toImageMsg();

  image_publisher.publish(msg);
}

bool TimepixHandler::setMode(int newmode) {

  if (mode == newmode) {

    return true;
  }

  ROS_INFO("Setting mode to %d", newmode);

  for (int i = 0; i < MATRIX_SIZE; i++) {
    if (newmode == 0)
      equalization[i] = (equalization[i] & 0b00111111);
    else
      equalization[i] = (equalization[i] | 0b01000000);
  }

  mode = newmode;

  if (!setEqualization()) {

    ROS_ERROR("Error during loading equalization while setting a mode.");
    return false;

  } else {
    return true;
  }
}

bool TimepixHandler::setModeCallback(rospix::SetMode::Request &req, rospix::SetMode::Response &res) {

  if (opened) {

    if (current_state != IDLE) {

      if (!setMode(req.mode)) {

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

    res.success = false;
    res.message = "We are already measuring.";
    return true;
  }

  exposure = req.exposure_time;

  current_state = CONTINOUS_EXPOSURE;

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

  if (current_state != IDLE) {

    res.success = false;
    res.message = "We are already measuring.";
    return true;
  }

  exposure = req.exposure_time;
  batch_exposure_count = req.exposure_count;

  current_state = BATCH_EXPOSURE;

  res.success = true;
  res.message = "Meaurement started.";
  return true;

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

bool TimepixHandler::setThresholdCallback(rospix::SetThreshold::Request &req, rospix::SetThreshold::Response &res) {

  if (req.threshold >= 0 && req.threshold <= 1000) {

    threshold = req.threshold;
    res.success = true;
    res.message = "Threshold changed.";
    return true;

  } else {

    res.success = false;
    res.message = "Threshold is out of bounds [0, 1000]";
    return true;
  }
}

bool TimepixHandler::setBiasCallback(rospix::SetBias::Request &req, rospix::SetBias::Response &res) {

  if (req.bias < 5 || req.bias > 94) {

    res.success = false;
    res.message = "Bias voltage out of bounds [5.4, 94] V.";
    return false;
  }

  bias = req.bias;

  if (!setNewBias(bias)) {

    res.success = false;
    res.message = "Error during setting new bias.";
    return false;
  }

  res.success = true;
  res.message = "New bias set.";
  return true;
}
