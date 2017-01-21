#ifndef TIMEPIX_H
#define TIMEPIX_H

/* author: Tomas Baca */
#include <ros/ros.h>
#include <string>
#include <rospix/SingleExposure.h>

// regarding image transport
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

#define MATRIX_SIZE 65536

typedef enum {

  USB_LITE,
  FITPIX

} InterfaceType_t;

enum {
  MPX,
  TOT
} SensorMode_t;

class TimepixHandler {

  public:

    // the constructor
    TimepixHandler(ros::NodeHandle nh);    

    // open the device
    bool open();

  private:

    // hold the interface type: usb lite / fitpix
    InterfaceType_t interface;

    // the string identifier of the sensor
    string name_;
    string custom_name_;

    // whether the device is openned
    bool opened;

    // chip id 
    int id; // by this id, the methods are let known about the interface
    string chip_id;

    // node handle
    ros::NodeHandle nh_;

    uint16_t dacs[14];
    uint16_t threshold;  
    double exposure;
    double bias;
    int mode;
    string equalization_file;
    bool equalization_loaded;

    uint16_t image[MATRIX_SIZE];
    uint8_t equalization[MATRIX_SIZE];

  private:

    bool loadDacs(); 
    bool setThreshold(const uint16_t newThreshold);
    bool setNewBias(const double newBias);
    bool loadEqualization(const string filename);
    bool doExposure(double time);
    bool readImage();
    bool publishImage();
    bool setEqualization();
    bool setMode(int newmode);

  private:

    ros::ServiceServer service_single_exposure;
    image_transport::Publisher image_publisher;

  private:

    bool singleExposureCallback(rospix::SingleExposure::Request &req, rospix::SingleExposure::Response &res);
};

#endif
