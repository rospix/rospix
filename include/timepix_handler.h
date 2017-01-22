#ifndef TIMEPIX_H
#define TIMEPIX_H

/* author: Tomas Baca */
#include <ros/ros.h>
#include <string>

// service messages
#include <rospix/Exposure.h>
#include <rospix/BatchExposure.h>
#include <rospix/SetMode.h>
#include <rospix/SetThreshold.h>
#include <rospix/SetBias.h>
#include <std_srvs/Trigger.h>

// regarding image transport
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <thread>

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

typedef enum {

  IDLE,
  SINGLE_EXPOSURE,
  CONTINOUS_EXPOSURE,
  BATCH_EXPOSURE,
  
} State_t;

class TimepixHandler {

  public:

    // the constructor
    TimepixHandler(ros::NodeHandle nh);    

    // open the device
    bool open(void);

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
    double batch_exposure_count;
    double bias;
    int mode;
    string equalization_file;
    bool equalization_loaded;
    bool exposing;

    uint16_t image[MATRIX_SIZE];
    uint8_t equalization[MATRIX_SIZE];

  private:

    bool loadDacs(void); 
    bool setThreshold(const uint16_t newThreshold);
    bool setNewBias(const double newBias);
    bool loadEqualization(const string filename);
    bool doExposure(double time);
    bool readImage(void);
    bool publishImage(void);
    bool setEqualization(void);
    bool setMode(int newmode);
    void doSingleExposure(void);

  private:
    
    bool dummy;
    bool dummy_simulate_focus;
    int dummy_photon_flux;
    double samplePseudoNormal(double mean, double std);
    void simulateExposure(void);
    double randf(double from, double to);
    int randi(int from, int to);

  private:

    ros::ServiceServer service_single_exposure;
    ros::ServiceServer service_continuous_exposure;
    ros::ServiceServer service_batch_exposure;
    ros::ServiceServer service_set_mode;
    ros::ServiceServer service_set_bias;
    ros::ServiceServer service_set_threshold;
    ros::ServiceServer service_interrupt_measurement;
    image_transport::Publisher image_publisher;

  private:

    std::thread main_thread;
    void mainThread(void);

  private:

    State_t current_state;
    void changeState(State_t new_state);

  private:

    bool singleExposureCallback(rospix::Exposure::Request &req, rospix::Exposure::Response &res);
    bool continuouExposureCallback(rospix::Exposure::Request &req, rospix::Exposure::Response &res);
    bool batchExposureCallback(rospix::BatchExposure::Request &req, rospix::BatchExposure::Response &res);
    bool setModeCallback(rospix::SetMode::Request &req, rospix::SetMode::Response &res);
    bool interruptMeasurementCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool setThresholdCallback(rospix::SetThreshold::Request &req, rospix::SetThreshold::Response &res);
    bool setBiasCallback(rospix::SetBias::Request &req, rospix::SetBias::Response &res);
};

#endif
