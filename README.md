# MXCarkit

The **MXCarkit** is the ideal platform for reasarch, developement and education in autonomous driving. For more information, please visit the official product [website](https://mdynamix.de/mx-academy-training/mxcarkit/).

<img src="images/mxcarkit_setup.jpeg" title="MXCarkit" width="1000">

</br>

## Initial Setup
Follow the instructions to setup [Jeston Nano](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) or [Jetson NX](https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit).

Clone this repository.
```
git clone https://github.com/william-mx/MXCarkit.git
```

Run shell script.
```
cd ./MXCarkit
chmod +x initial_setup_jetson_nano.sh
./initial_setup_jetson_nano.sh
```

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


