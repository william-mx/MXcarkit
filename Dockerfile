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
    ros-$ROS_DISTRO-joy 
    # ros-$ROS_DISTRO-realsense2-camera
    # Unable to read IMU data on the D435i
    # realsense2-camera build from source instead
    # see http://wiki.ros.org/realsense_camera/Tutorials/Building_from_Sources
    # see https://github.com/IntelRealSense/librealsense/issues/10304#issuecomment-1067334337


WORKDIR ./catkin_ws

RUN . /opt/ros/melodic/setup.sh 



