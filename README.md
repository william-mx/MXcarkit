# MXCarkit
<<<<<<< HEAD

The **MXCarkit** is the ideal platform for reasarch, developement and education in autonomous driving. For more information, please visit the official product [website](https://mdynamix.de/mx-academy-training/mxcarkit/).

<img src="images/mxcarkit_setup.jpeg" title="MXCarkit" width="800">

</br>

## Initial Setup
Follow the instructions to setup [Jeston Nano](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) or [Jetson NX](https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit).
</br>

## Getting Started

</br>

Run docker image
```
sudo docker run -it --rm \
--mount type=bind,source=/home/mxck/mxck_ws/mxck_hw,target=/catkin_ws \
--privileged \
--net=host \
--name ros_melodic_docker \
mxck_ros_hw:latest bash
```
=======
 
<img src="images/mxcarkit_setup.jpeg" title="MXCarkit" width="1000">
>>>>>>> e8e1f874550185e4ea698f3220179a9f61a51e11
