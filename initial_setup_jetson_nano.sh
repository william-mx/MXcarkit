#!/bin/bash

# set permission to access serial devices
usermod -a -G dialout mxck

# remove libreoffice to save diskspace
apt-get update &&\
apt-get autoremove libreoffice* -y &&\
apt-get clean

# install relasense viewer
# see https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation for more details
apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE &&\
add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u &&\
apt-get install --yes librealsense2-utils &&\
apt-get install --yes librealsense2-dev

# install fan control
git clone https://github.com/Pyrestone/jetson-fan-ctl.git &&\
cd jetson-fan-ctl/ &&\
./install.sh

# bin usb devices under static name unsing udev rules
# see http://reactivated.net/writing_udev_rules.html
echo -e \
'SUBSYSTEM=="tty", ACTION=="add", ATTRS{manufacturer}=="STMicroelectronics", ATTRS{idVendor}=="0483", ATTRS{product}=="STM32 STLink", ATTRS{idProduct}=="374b", MODE="777", SYMLINK+="stm32_nucleo"
SUBSYSTEM=="tty", ACTION=="add", ATTRS{manufacturer}=="STMicroelectronics", ATTRS{idVendor}=="0483", ATTRS{product}=="ChibiOS/RT Virtual COM Port", ATTRS{idProduct}=="5740", MODE="777", SYMLINK+="vesc"' \
>> /etc/udev/rules.d/10-local.rules

# build docker image
docker build -t mxck_ros_hw .

# set the permissions the X server host
# see http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:docker

# move ros workspace to home directory
mv mxck_ws ~/
