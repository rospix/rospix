#ifndef TIMEPIX_H
#define TIMEPIX_H

/* author: Tomas Baca */
#include <ros/ros.h>
#include <string>

// service messages
#include <rospix/BatchExposure.h>
#include <rospix/Exposure.h>
#include <rospix/Image.h>
#include <rospix/SetDouble.h>
#include <rospix/SetInt.h>
#include <std_srvs/Trigger.h>

#include <thread>

using namespace std;

#define MATRIX_SIZE 65536

typedef enum
{

  USB_LITE,
  FITPIX

} InterfaceType_t;

enum
{
  MPX,
  TOT
} SensorMode_t;

typedef enum
{

  IDLE,
  SINGLE_EXPOSURE,
  CONTINOUS_EXPOSURE,
  BATCH_EXPOSURE,

} State_t;

class TimepixHandler {

public:
  // the constructor
  TimepixHandler(ros::NodeHandle nh, string idname, string equalization_directory);

  // open the device
  bool open(void);

private:
  // hold the interface type: usb lite / fitpix
  InterfaceType_t interface;

  // the string identifier of the sensor
  string name_;
  string idname_;

  // whether the device is openned
  bool opened;

  // chip id
  int    id;  // by this id, the methods are let known about the interface
  string chip_id;

  // node handle
  ros::NodeHandle nh_;
  string          equalization_directory;

  uint16_t dacs[14];
  uint16_t threshold;
  double   exposure;
  int      batch_exposure_count;
  double   bias;
  int      mode;
  int      clock;
  string   equalization_file;
  bool     equalization_loaded;
  bool     exposing;

  uint16_t  image[MATRIX_SIZE];
  uint8_t   equalization[MATRIX_SIZE];
  double    time_total;
  ros::Time exposure_started;
  double    total_exposing_time;
  bool      print_utilization_;

  rospix::Image outputImage;

private:
  bool loadDacs(void);
  bool setThreshold(const uint16_t newThreshold);
  bool setNewBias(const double newBias);
  bool loadEqualization(void);
  bool doExposure(double time);
  bool readImage(void);
  void publishImage(void);
  bool setEqualization(void);
  bool setNewClock(const int new_clock);
  bool setMode(int newmode);
  bool setClock(int newclock);
  void doSingleExposure(void);
  bool reOpen(void);
  void interruptMeasurement(void);

private:
  bool   dummy;
  int    dummy_photon_flux_;
  double samplePseudoNormal(double mean, double std);
  void   simulateExposure(void);
  double randf(double from, double to);
  int    randi(int from, int to);
  int    dummy_counter;
  bool   simulate_background_;
  int    dummy_n_images_;
  int    optics_dimension_;

private:
  ros::ServiceServer service_single_exposure;
  ros::ServiceServer service_continuous_exposure;
  ros::ServiceServer service_batch_exposure;
  ros::ServiceServer service_set_mode;
  ros::ServiceServer service_set_exposure;
  ros::ServiceServer service_set_bias;
  ros::ServiceServer service_set_clock;
  ros::ServiceServer service_set_threshold;
  ros::ServiceServer service_interrupt_measurement;

  ros::Publisher image_publisher;

private:
  std::thread main_thread;
  void        mainThread(void);

private:
  State_t current_state;
  void    changeState(State_t new_state);

private:
  bool singleExposureCallback(rospix::Exposure::Request &req, rospix::Exposure::Response &res);
  bool continuouExposureCallback(rospix::Exposure::Request &req, rospix::Exposure::Response &res);
  bool batchExposureCallback(rospix::BatchExposure::Request &req, rospix::BatchExposure::Response &res);
  bool setModeCallback(rospix::SetInt::Request &req, rospix::SetInt::Response &res);
  bool interruptMeasurementCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool setThresholdCallback(rospix::SetInt::Request &req, rospix::SetInt::Response &res);
  bool setBiasCallback(rospix::SetDouble::Request &req, rospix::SetDouble::Response &res);
  bool setExposureCallback(rospix::SetDouble::Request &req, rospix::SetDouble::Response &res);
  bool setClockCallback(rospix::SetInt::Request &req, rospix::SetInt::Response &res);
};

#endif
