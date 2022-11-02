FROM dustynv/ros:melodic-ros-base-l4t-r32.4.4

RUN apt-get update \
 && apt-get install --yes \
    ros-$ROS_DISTRO-tf \
    ros-$ROS_DISTRO-serial \
    ros-$ROS_DISTRO-rosserial \
    ros-$ROS_DISTRO-rosserial-python \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-rplidar-ros \
    ros-$ROS_DISTRO-image-view \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-realsense2-camera

# install Intel RealSense SDK 2.0
# see https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation for more details
# required to receive imu data
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 

RUN apt-get install --yes software-properties-common

RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u 

RUN apt-get install --yes \
    librealsense2-utils \
    librealsense2-dev

WORKDIR ./catkin_ws

RUN . /opt/ros/melodic/setup.sh 



