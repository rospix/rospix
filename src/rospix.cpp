// some ros includes
#include <ros/ros.h>
#include <ros/package.h>

// some std includes
#include <stdlib.h>
#include <stdio.h>

// some opencv includes
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <timepix_handler.h>

#include <rospix/Status.h>

#include <usb.h>

using namespace std;
using namespace cv;

// global variable for storing the image
Mat frame_global;

// pointer to OpenCV bridge
cv_bridge::CvImage cv_ptr;;

list<TimepixHandler *> sensors;

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "rospix");
  ros::NodeHandle nh_ = ros::NodeHandle("~");
  ros::Publisher status = nh_.advertise<rospix::Status>("status", 1);

  int number_of_detectors;

  nh_.param("number_of_detectors", number_of_detectors, 0);

  // list the devices, this is neccessary to open them
  // const char* devNames[50];
  // int devCount = 0;
  // listDevices(devNames, &devCount);

  // iterate over all the detectors in the config file and try to open them
  for (int i = 0; i < number_of_detectors; i++) {

    char id_char[10];
    sprintf(id_char, "sensor_%d", i);
    string id_str = string(id_char);

    TimepixHandler * sensor = new TimepixHandler(ros::NodeHandle(string("~/")+id_str));

    if (!sensor->open()) {
      ROS_WARN("Failed to open \"%s\"", id_str.c_str());
    } else {  
      sensors.push_back(sensor);
    }
  }

  rospix::Status newStatus;
  newStatus.stamp = ros::Time::now();
  newStatus.status_code = sensors.size();

  char temp[30];
  sprintf(temp, "ROSpix started with %d detectors.", int(sensors.size()));
  string msg = string(temp);
  newStatus.message = msg;

  if (sensors.size() == number_of_detectors) {
    ROS_INFO_STREAM(msg);
  } else {
    ROS_WARN_STREAM(msg);
  }

  status.publish(newStatus);

  ros::Duration d(2);
  d.sleep();

  ros::spin();

  return 0;
}
