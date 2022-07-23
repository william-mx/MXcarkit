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