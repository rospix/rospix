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
  memset(equalization, 0, MATRIX_SIZE * sizeof(uint8_t));

  this->nh_ = nh;

  nh_.param("name", name_, string());
  nh_.param("custom_name", custom_name_, string());

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

  nh_.param("defaults/exposure", exposure, 0.0);

  if (exposure == 0.0) {
    ROS_ERROR("Error loading the default exposure time from the config file.");
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

  memset(image, 0, MATRIX_SIZE * sizeof(uint16_t));

  ROS_INFO("Initializing sensor \"%s\" with custom name \"%s\".", name_.c_str(), custom_name_.c_str());
}

bool TimepixHandler::open() {

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

  if (opened) {

    service_single_exposure = nh_.advertiseService("singleExposure", &TimepixHandler::singleExposureCallback, this);
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
  }

  return opened;
}

bool TimepixHandler::singleExposureCallback(rospix::SingleExposure::Request &req, rospix::SingleExposure::Response &res) {

  if (req.exposure_time <= 0) {

    res.success = false;
    res.message = "The exposure time should be positive.";
    return true;
  }

  if (!doExposure(req.exposure_time)) {

    ROS_ERROR("Error while doing an exposure.");
    res.success = false;
    res.message = "Error";
  } else {

    if (!readImage()) {

      ROS_ERROR("Could not read the image.");

    } else {

      publishImage();
      res.success = true;
      res.message = "Exposured done";
      return true;
    }
  }

  res.success = false;
  res.message = "Some error";
  return true;
}

bool TimepixHandler::loadDacs() {

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

  dacs[6] = newThreshold;

  return loadDacs();
}

bool TimepixHandler::setNewBias(const double newBias) {

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

  std::fstream myfile(filename.c_str(), std::ios_base::in);

  if (!myfile.is_open()) {

    ROS_ERROR("Cannot read the equalization!");
    return false;
  }

  uint16_t th0, th1, th2, th3, msk, tst;
  int tempByte;
  uint16_t pixelCfg;
  int idx = 0;
  while (myfile >> tempByte) {

    equalization[idx] = '0'+uint8_t(tempByte);
    equalization[idx] = equalization[idx] | 0b00000001;

    idx++;

    if (idx > MATRIX_SIZE) {

      ROS_WARN("Equalization file is larger than it should!");
      break;
    }
  }

  // check if we fill the whole matrix
  if (idx < MATRIX_SIZE) {

    ROS_WARN("Eqalization in the file is smaller then it should!");
  }

  equalization_loaded = true;

  return true;
}

bool TimepixHandler::setEqualization() {

  int rc = 0;

  if (!equalization_loaded) {

    ROS_ERROR("Cannot set the equalization, matrix not loaded.");
    return false;
  }

  /*
     for (int i = 0; i < 10; i++) {
     ROS_INFO("%d: %d", i, equalization[i]);
     }
     */

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

bool TimepixHandler::readImage() {

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

bool TimepixHandler::publishImage() {

  cv::Mat cv_image = cv::Mat::zeros(256, 256, CV_16UC1);

  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 256; j++) {

      cv_image.at<uint16_t>(i, j) = (image[j*256 + i] > 0) ? 60000 : 0; 
    }
  }

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", cv_image).toImageMsg();

  image_publisher.publish(msg);
}

bool TimepixHandler::setMode(int newmode) {

  if (mode == newmode) {

    return true;
  }

  for (int i = 0; i < MATRIX_SIZE; i++) {
    if (mode == MPX)
      equalization[i] = (equalization[i] & 0b10111111);
    else
      equalization[i] = (equalization[i] | 0b01000000);
  }

  if (!setEqualization()) {

    ROS_ERROR("Error during loading equalization while setting a mode.");
    return false;

  } else {
    return true;
  }
}
