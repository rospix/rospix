#include <ros/ros.h>
#include "../include/timepix.h"
#include "../include/ftd2xx.h"

// the constructor
TimepixHandler::TimepixHandler(void) {

  iNumDevs = 0;

  int i;

  for(int i = 0; i < MAX_DEVICES; i++) {
    pcBufLD[i] = cBufLD[i];
  }

  pcBufLD[MAX_DEVICES] = NULL; 

  ROS_INFO("mys");
  
  // list all ftdi devices
  ftStatus = FT_ListDevices(pcBufLD, &iNumDevs, FT_LIST_ALL | FT_OPEN_BY_SERIAL_NUMBER);

  ROS_INFO("kocka");

  if (ftStatus != FT_OK) {
    ROS_INFO("Error: FT_ListDevices(%d)\n", (int) ftStatus);
  }

  for (i = 0; ( (i <MAX_DEVICES) && (i < iNumDevs) ); i++) {
    ROS_INFO("Device %d Serial Number - %s\n", i, cBufLD[i]);
  }

}
