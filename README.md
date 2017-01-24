# ROSPix
ROS package for working with Timepix sensor.

# Dummy detectors

The node allows creating ("connecting") dummy detectors. Any number of dummy detectors
can be connected, even while real detectors are present. Here is an example of **config file**
record for an dummy detector:
```
sensor_1:

  name: 'dummy'

  simulate_focus: true        # should we simulate optics?
  optics_dimension: 2         # optics dimensionality 1=1D, 2=2D
  photon_flux: 100            # [photons/s], whole number
  simulate_background: true   # should we simulation radiation background?
  n_images: 20                # how many images of radiation background do we have?

  defaults:
    exposure: 1.0             # [seconds]
```
Images for creating artifitial radiation background are located in the **dummy** subfolder. If anyone should provide their own additional images, copy them there and adjust **n_images** accordingly.

# Prerequsities

## FTDI drivers

Get the drivers:

```bash
sudo apt-get install libftdi-*
```

Create file **99-ftdi-sio.rules** with following line
```bash
ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", MODE="0666",  RUN+="/bin/sh -c '/sbin/rmmod ftdi_sio && /sbin/rmmod usbserial'"
ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666",  RUN+="/bin/sh -c '/sbin/rmmod ftdi_sio && /sbin/rmmod usbserial'"
```
and place it in /etc/udev/rules.d/

## Installing ROS

Follow tutorials on http://wiki.ros.org/kinetic/Installation/Ubuntu ... the cores is extracted in following commands:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full python-catkin-tools
apt-cache search ros-kinetic
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
