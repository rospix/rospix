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

    return true;
  }

  return false;
}
