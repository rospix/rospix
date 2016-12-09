# ROSPix
ROS package for working with Timepix sensor.


# 99-ftdi-sio.rules
```bash
ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", MODE="0666",  RUN+="/bin/sh -c '/sbin/rmmod ftdi_sio && /sbin/rmmod usbserial'"
```
