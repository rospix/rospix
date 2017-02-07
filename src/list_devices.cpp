// some ros includes
#include <ros/ros.h>
#include <ros/package.h>

// some std includes
#include <stdlib.h>
#include <stdio.h>

// timepix library
#include "usb.h"

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "list_devices");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  //////////////////// List Usb Lite devices \\\\\\\\\\\\\\\\\\\\\

  const char* devNames[50];
  int devCount = 0;

  listDevices(devNames, &devCount);

  for (int i = 0; i < devCount; i++) {

    ROS_INFO("USB Lite n.%d: \"%s\"", i+1, devNames[i]);
  }

  //////////////////// List Fitpix devices \\\\\\\\\\\\\\\\\\\\\

  listDevicesFpx(devNames, &devCount);

  for (int i = 0; i < devCount; i++) {

    ROS_INFO("FitPix n.%d: \"%s\"", i+1, devNames[i]);
  }

  ros::shutdown();

  return 0;
}
