// some ros includes
#include <ros/ros.h>
#include <ros/package.h>

// some std includes
#include <stdlib.h>
#include <stdio.h>

#include "usb.h"

// use gui?
bool gui = true;

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "list_devices");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  // List USB Lite devices

  const char* devNames[50];
  int devCount = 0;

  listDevices(devNames, &devCount);

  for (int i = 0; i < devCount; i++) {

    ROS_INFO("USB LIte n.%d: %s", i+1, devNames[i]);
  }

  // List FitPix devices

  listDevicesFpx(devNames, &devCount);

  for (int i = 0; i < devCount; i++) {

    ROS_INFO("FitPix n.%d: \"%s\"", i+1, devNames[i]);
  }


  return 0;
}
