/**
 * Copyright (C) 2017 Daniel Turecek
 *
 * @file      usb.h
 * @author    Daniel Turecek <daniel@turecek.de>
 * @date      2017-01-13
 *
 */
#ifndef USB_H
#define USB_H
#include "common.h"


EXPORTFUNC int listDevices(const char* devnames[50], int* count);
EXPORTFUNC int listDevicesFpx(const char* devnames[50], int* count);
EXPORTFUNC int openDevice(const char* devName, int* id);
EXPORTFUNC int openDeviceFpx(const char* devName, int* id);
EXPORTFUNC int setDacs(int id, u16* dacs);
EXPORTFUNC int setDacsFpx(int id, u16* dacs);
EXPORTFUNC int writePixCfg(int id, byte* cfg, size_t size);
EXPORTFUNC int writePixCfgFpx(int id, byte* cfg, size_t size);
EXPORTFUNC int readMatrix(int id, u16* matrix, int size, bool convertPseudo);
EXPORTFUNC int readMatrixFpx(int id, u16* matrix, int size, bool convertPseudo);
EXPORTFUNC int doAcquisition(int id, double acqTime);
EXPORTFUNC int doAcquisitionFpx(int id, double acqTime);
EXPORTFUNC int abortAcquisition(int id);
EXPORTFUNC int abortAcquisitionFpx(int id);
EXPORTFUNC int setBias(int id, double bias);
EXPORTFUNC int setBiasFpx(int id, double bias);
EXPORTFUNC int setTpxClock(int id, double clock);
EXPORTFUNC int setTpxClockFpx(int id, double clock);
EXPORTFUNC const char* chipID(int id);
EXPORTFUNC const char* chipIDFpx(int id);
EXPORTFUNC const char* lastError(int id);
EXPORTFUNC const char* lastErrorFpx(int id);


#endif /* !USB_H */

