# ROSPix
ROS package for working with Timepix sensor.

# FTDI drivers

Get the drivers:

For Odroid (ARMv7):
```bash
cd /tmp
wget http://www.ftdichip.com/Drivers/D2XX/Linux/libftd2xx-arm-v7-hf-1.3.6.tgz
```

For x64:
```bash
cd /tmp
wget http://www.ftdichip.com/Drivers/D2XX/Linux/libftd2xx-x86_64-1.3.6.tgz
```

Install them:
```bash
cd /tmp/release/build
sudo cp libftd2xx.* /usr/local/lib
sudo chmod 0755 /usr/local/lib/libftd2xx.so.1.3.6
sudo ln -sf /usr/local/lib/libftd2xx.so.1.3.6 /usr/local/lib/libftd2xx.so
sudo ldconfig -v 
```

create file 99-ftdi-sio.rules with following line
```bash
ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", MODE="0666",  RUN+="/bin/sh -c '/sbin/rmmod ftdi_sio && /sbin/rmmod usbserial'"
```
and place it in /etc/udev/rules.d/

# Installing ROS

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
apt-cache search ros-kinetic
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# Installing prerequisities

sudo apt-get install python-catkin-tools

