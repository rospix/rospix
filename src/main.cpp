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

#include <timepix.h>

using namespace std;
using namespace cv;

// global variable for storing the image
Mat frame_global;

// pointer to OpenCV bridge
cv_bridge::CvImage cv_ptr;

// use gui?
bool gui = true;

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "rospix");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  // load config
  nh_.param("gui", gui, bool(true));

  // lets tell the world we are read
  ROS_INFO("ROSPix node initialized");

  TimepixHandler * timepix = new TimepixHandler(nh_);

  // infinite loop
  while (ros::ok()) {

    ros::spinOnce();
  }
}
