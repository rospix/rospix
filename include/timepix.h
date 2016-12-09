#ifndef TIMEPIX_H
#define TIMEPIX_H

/* author: Tomas Baca */
#include <ros/ros.h>
#include "../include/ftd2xx.h"
#include "../include/WinTypes.h"

#define BUF_SIZE      1
#define MAX_DEVICES   5

class TimepixHandler {

  public:

    // the constructor
    TimepixHandler(void);

  private:

    int	iNumDevs;
    char *  pcBufLD[MAX_DEVICES + 1];
    char  cBufLD[MAX_DEVICES][64];
    FT_STATUS	ftStatus;
    FT_HANDLE	ftHandle[MAX_DEVICES];
};

#endif
