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

  // lets tell the world we are read
  ROS_INFO("ROSPix node initialized");

  int number_of_detectors;
  
  nh_.param("number_of_detectors", number_of_detectors, 0);
 
  // iterate over all the detectors in the config file and try to open them
  for (int i = 0; i < number_of_detectors; i++) {

    char id_char[10];
    sprintf(id_char, "Sensor_%d", i);
    string id_str = string(id_char);

    TimepixHandler * sensor = new TimepixHandler(ros::NodeHandle(string("~/")+id_str));

    if (!sensor->open()) {
       ROS_WARN("Failed to open \"%s\"", id_str.c_str());
    } else {  
      sensors.push_back(sensor);
    }
  }

  ros::spin();

  return 0;
}
