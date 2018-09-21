// some ros include
#include <ros/package.h>
#include <ros/ros.h>

// some std includes
#include <stdio.h>
#include <stdlib.h>

// some opencv includes
#include <ros/package.h>

#include "timepix_handler.h"

#include <rospix/Status.h>

#include <usb.h>

using namespace std;

list<TimepixHandler*> sensors;

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "rospix");
  ros::NodeHandle nh_    = ros::NodeHandle("~");
  ros::Publisher  status = nh_.advertise<rospix::Status>("status", 1);

  int    number_of_detectors;
  string equalization_directory;

  nh_.param("number_of_detectors", number_of_detectors, 0);
  nh_.param("equalization_directory", equalization_directory, ros::package::getPath("rospix") + string("/equalizations"));

  // list the devices, this is neccessary to open them
  // const char* devNames[50];
  // int devCount = 0;
  // listDevices(devNames, &devCount);

  // iterate over all the detectors in the config file and try to open them
  for (int i = 0; i < number_of_detectors; i++) {

    char id_char[10];
    sprintf(id_char, "sensor_%d", i);
    string id_str = string(id_char);

    TimepixHandler* sensor = new TimepixHandler(ros::NodeHandle(string("~/") + id_str), id_str, equalization_directory);

    if (!sensor->open()) {
      ROS_WARN("Failed to open \"%s\"", id_str.c_str());
    } else {
      sensors.push_back(sensor);
    }
  }

  rospix::Status newStatus;
  newStatus.stamp       = ros::Time::now();
  newStatus.status_code = sensors.size();

  char temp[100];
  sprintf(temp, "rospix started with %d detectors.", int(sensors.size()));
  string msg        = string(temp);
  newStatus.message = msg;

  if (int(sensors.size()) == number_of_detectors) {
    ROS_INFO_STREAM(msg);
  } else {
    ROS_WARN_STREAM(msg);
  }

  status.publish(newStatus);

  ros::AsyncSpinner spinner(number_of_detectors);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
