#include <ros/ros.h>
#include "../include/timepix.h"
#include "../include/ftd2xx.h"

// the constructor
TimepixHandler::TimepixHandler(ros::NodeHandle nh) {

  iNumDevs = 0;

  int i;

  for(int i = 0; i < MAX_DEVICES; i++) {
    pcBufLD[i] = cBufLD[i];
  }

  pcBufLD[MAX_DEVICES] = NULL; 

  // list all ftdi devices
  ftStatus = FT_ListDevices(pcBufLD, &iNumDevs, FT_LIST_ALL | FT_OPEN_BY_SERIAL_NUMBER);

  if (ftStatus != FT_OK) {
    ROS_INFO("Error: FT_ListDevices(%d)", (int) ftStatus);
  }

  for (i = 0; ( (i <MAX_DEVICES) && (i < iNumDevs) ); i++) {
    ROS_INFO("Device %d at Number - %s", i, cBufLD[i]);
  }

  int device = 0;

  if ((ftStatus = FT_OpenEx(cBufLD[device], FT_OPEN_BY_SERIAL_NUMBER, &ftHandle[device])) != FT_OK) {
    
    ROS_INFO("Error FT_OpenEx(%d), device %d", (int) ftStatus, i);
    ROS_INFO("Use lsmod to check if ftdi_sio (and usbserial) are present.");
    ROS_INFO("If so, unload them using rmmod, as they conflict with ftd2xx.");
  }
		
  ROS_INFO("Opened device %s", cBufLD[device]);

  // write to medipix
  
  char buffer[256];
  for (int i = 0; i < 256; i++) {
    buffer[i] = 0;
  }
  buffer[0] = '_';
  DWORD bytestotransmit = 1;
  DWORD BytesWritten = 0;

  ftStatus = FT_Write(ftHandle[device], &buffer, bytestotransmit, &BytesWritten);
  ftStatus = FT_Write(ftHandle[device], &buffer, bytestotransmit, &BytesWritten);
  ftStatus = FT_Write(ftHandle[device], &buffer, bytestotransmit, &BytesWritten);

  ROS_INFO_STREAM("Bytes written: " << BytesWritten);

  DWORD rxbytes, txbytes, eventdword;

  ros::Duration sec(5);
  sec.sleep();

  FT_GetStatus(ftHandle[device], &rxbytes, &txbytes, &eventdword);

  ROS_INFO_STREAM(rxbytes << " " << txbytes << " " << eventdword);

  // while (true) {
  // 
  //   DWORD BytesReceived = 0;
  //   ftStatus = FT_Read(ftHandle[device], &buffer, bytestotransmit, &BytesReceived);

  //   ROS_INFO("pes");

  //   if (BytesReceived > 0) {
  //     ROS_INFO_STREAM(buffer[0]);
  //   } else 
  //     break;
  // }

  ROS_INFO("done");
}

