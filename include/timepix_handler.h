#ifndef TIMEPIX_H
#define TIMEPIX_H

/* author: Tomas Baca */
#include <ros/ros.h>
#include <string>

using namespace std;

typedef enum {

  USB_LITE,
  FITPIX

} InterfaceType_t;


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

  private:

    bool loadDacs(); 
    bool setThreshold(const uint16_t newThreshold);
    bool setNewBias(const double newBias);
};

#endif
