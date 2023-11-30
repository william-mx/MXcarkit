#!/bin/bash

# set up a hotspot with the name 'mxck' and the password 'mxck1234'
sudo nmcli dev wifi hotspot ifname wlan0 ssid mxck password mxck1234

# run docker in new terminal
sleep 5
gnome-terminal --tab -- "/home/mxck/mxck_ws/mxck_base/run_ros_docker.sh" 'true'









