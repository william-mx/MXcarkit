# MXCarkit

The **MXCarkit** is the ideal platform for reasarch, developement and education in autonomous driving. For more information, please visit the official product [website](https://mdynamix.de/mx-academy-training/mxcarkit/).

<img src="images/mxcarkit_setup.jpeg" title="MXCarkit" width="1000">

</br>

## Initial Setup
Follow the instructions to setup [Jeston Nano](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) or [Jetson NX](https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit).

Clone this repository.
```
git clone --recurse-submodules https://github.com/william-mx/MXCarkit.git
```

Run shell script.
```
cd ./MXCarkit
chmod +x initial_setup.sh
./initial_setup.sh
```



</br>

## Getting Started

</br>

Make shell script executable.
```
cd ./mxck_ws
chmod +x run_ros_docker.sh add_ros_docker.sh 
```

Run docker image
```
./run_ros_docker.sh
```

Add additional docker terminal
```
./add_ros_docker.sh
```


Launch hardware - VESC, LEDs, USS, IMU, LIDAR, CAMERA

```
source devel/setup.bash
roslaunch mxck_run mxck_run.launch
```

Add additional docker terminal
```
sudo docker ps
sudo docker exec -it <container_id> bash
```