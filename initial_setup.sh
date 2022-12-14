#!/bin/bash


# store current working directory in a variable
cwd=$(pwd)


# device: nano, nx
while getopts d: flag
do
    case "${flag}" in
        d) device=${OPTARG};;
    esac
done

# set permission to access serial devices
usermod -a -G dialout mxck

# remove libreoffice to save diskspace
apt-get update &&\
apt-get autoremove libreoffice* -y &&\
apt-get clean

# install curl
apt-get install --yes curl

# install relasense viewer
# see https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation for more details
apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE &&\
add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u &&\
apt-get install --yes librealsense2-utils &&\
apt-get install --yes librealsense2-dev


# bin usb devices under static name unsing udev rules
# see http://reactivated.net/writing_udev_rules.html
echo -e \
'SUBSYSTEM=="tty", ACTION=="add", ATTRS{manufacturer}=="STMicroelectronics", ATTRS{idVendor}=="0483", ATTRS{product}=="STM32 STLink", ATTRS{idProduct}=="374b", MODE="777", SYMLINK+="stm32_nucleo" # ST-Link
SUBSYSTEM=="tty", ACTION=="add", ATTRS{manufacturer}=="SEGGER", ATTRS{idVendor}=="1366", ATTRS{product}=="J-Link", ATTRS{idProduct}=="0105", MODE="777", SYMLINK+="stm32_nucleo" # J-Link
SUBSYSTEM=="tty", ACTION=="add", ATTRS{manufacturer}=="STMicroelectronics", ATTRS{idVendor}=="0483", ATTRS{product}=="ChibiOS/RT Virtual COM Port", ATTRS{idProduct}=="5740", MODE="777", SYMLINK+="vesc"' \
>> /etc/udev/rules.d/10-local.rules


# clone mxck workspace
git clone https://github.com/william-mx/mxck_ws.git

# move ros workspace to home directory
mv mxck_ws ~/

# build docker image
docker build -t mxck_ros_hw .


# set the permissions the X server host
# see http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:docker


# install fan control for jetson nano
if [ $device == "nano" ]
then
   cd ~/
   git clone https://github.com/Pyrestone/jetson-fan-ctl.git &&\
   cd ~/jetson-fan-ctl/ &&\
   cd $pwd
fi
