# ROSPix

ROS package for working with Timepix detectors.
Allows to conduct measurement with multiple sensors.
To be able to use it, install Linux OS compatible with ROS (Indigo or newer).
Install **ROS** and FTDI drivers, see section **Prerequisities**.

## Connecting to Timepix interface

Firstly, find a unique ID of your Timepix interface. Connect it to your computer and run:
```bash
roslaunch rospix list_device
```
A list of device names will appear.
Copy the insides of double commas, corresponding to your interface.
It is supposed to look similar to: **Medipix2 ver 1.32 sn: 1096**.
Create a config yaml (or modify the example in **config/**) file with following contents:

```
# dont forget to update this number according to following contents
number_of_detectors: 1

sensor_0: # the names are fixed, just the number change

  # by this name the device is found, run "roslaunch rospix list_devices" to find it
  name: 'Medipix2 ver 1.32 sn: 1096'

  # location of the equalization file
  # location is relative to a directory specified in launch file
  equalization: 'lite_rigaku.bpc' # *.bpc is needed
  
  # print percentage of sensor exposure time to real time
  print_utilization: true
  
  defaults:
    threshold: 340    # [-]
    bias: 10.0        # [Volt]
    exposure: 1.0     # [second]
    mode: 1           # [0 = MPX, 1 = TOT]

    # dacs can be found in calibration protocol (or equalization protocol)
    dacs: [1,100,255,127,127,0,340,7,130,128,80,85,128,128]
```

Secondly, place a pixel configuration matrix (equalization matrix) into **equalizations/** folder.

Lastly launch the node using launch file example **test.launch** in **launch/**.
It loads the example config file and sets the equalization directory.
```bash
roslaunch rospix test.launch
```

```
<launch>

  <!-- launch the node -->
  <node name="rospix" pkg="rospix" type="rospix" output="screen">

    <!-- load config from config file -->
    <rosparam file="$(find rospix)/config/test.yaml" />

    <!-- specify where should the node look for equalization matrices -->
    <param name="equalization_directory" value="$(find rospix)/equalizations/" />

    <!-- PUBLISHERS -->
    <remap from="~status" to="~status" />

  </node>

</launch>
```

### Dummy detectors

The node allows creating ("connecting") dummy detectors. Any number of dummy detectors
can be connected, even while real detectors are present. Here is an example of **config file**
record for an dummy detector:
```
sensor_1:

  name: 'dummy'

  photon_flux: 100            # [photons/s], whole number
  simulate_background: true   # should we simulation radiation background?
  n_images: 20                # how many images of radiation background do we have?

  defaults:
    exposure: 1.0             # [seconds]
```
Images for creating artificial radiation background are located in the **dummy** subfolder. If anyone should provide their own additional images, copy them there and adjust **n_images** accordingly.

## Interacting with the interfaces

After connecting to the devices, a message is published on
```
/rospix/status
```
reporting how many interfaces have been successfully opened.
Namespaces will appeare for the opened, named as follows:
```
/rospix/sensor_0/...
/rospix/sensor_1/...
...
```

Following services provide control over some basic operations:
```
/rospix/sensor_0/do_batch_exposure
/rospix/sensor_0/do_continuous_exposure
/rospix/sensor_0/do_single_exposure
/rospix/sensor_0/interrupt_measurement
/rospix/sensor_0/set_bias
/rospix/sensor_0/set_exposure_time
/rospix/sensor_0/set_mode
/rospix/sensor_0/set_threshold
...
/rospix/sensor_1/...
```

Resulting images are published at
```
/rospix/sensor_0/image
/rospix/sensor_...
```
as a topic containing:
```
time stamp
float64 exposure_time
float64 bias
int32 threshold
string interface
int32 mode
int16[65536] image
```

# Prerequsities

## FTDI drivers

Get the drivers:

```bash
sudo apt-get install libftdi-* ros-kinetic-cv-bridge
```

Create file **99-ftdi-sio.rules** with following lines
```bash
ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", MODE="0666",  RUN+="/bin/sh -c '/sbin/rmmod ftdi_sio && /sbin/rmmod usbserial'"
ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666",  RUN+="/bin/sh -c '/sbin/rmmod ftdi_sio && /sbin/rmmod usbserial'"
ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666",  RUN+="/bin/sh -c '/sbin/rmmod ftdi_sio && /sbin/rmmod usbserial'"
```
and place it in /etc/udev/rules.d/

## Installing ROS

Follow tutorials on http://wiki.ros.org/kinetic/Installation/Ubuntu ... the core is extracted in following commands:

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
