#include <ros/ros.h>

#include "timepix_handler.h"
#include <string>

// timepix driver headers
#include "usb.h"

using namespace std;

// the constructor
TimepixHandler::TimepixHandler(ros::NodeHandle nh) {

  opened = false;

  this->nh_ = nh;

  nh_.param("name", name_, string());
  nh_.param("custom_name", custom_name_, string());

  // load defaults from the config

  int tempint = 0;
  nh_.param("defaults/threshold", tempint, 0);
  threshold = uint16_t(tempint);

  if (threshold == 0) {
    ROS_INFO("Error loading the default threshold from config file.");
  }

  nh_.param("defaults/bias", bias, 0.0);

  if (bias == 0) {
    ROS_INFO("Error loading the default bias from the config file.");
  }

  nh_.param("defaults/exposure", exposure, 0.0);

  if (exposure == 0.0) {

    ROS_INFO("Error loading the default exposure time from the config file.");
  }

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

    opened = true;

    interface = USB_LITE;  

    loadDacs();

    return true;
  }

  // try to open fitpix device
  listDevicesFpx(devNames, &devCount);

  error = openDeviceFpx(name_.c_str(), &id);

  // success?
  if (error == 0) {

    chip_id = chipID(id);

    ROS_INFO("Successfully opened FitPix \"%s\", its chip is is \"%s\".", name_.c_str(), chip_id.c_str());

    opened = true;

    interface = FITPIX;

    loadDacs();

    return true;
  }

  return false;
}

bool TimepixHandler::loadDacs() {

  int rc = 0;

  switch (interface) {

    case USB_LITE:

      rc = setDacs(id, dacs);
      break;

    case FITPIX:

      rc = setDacs(id, dacs);
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
