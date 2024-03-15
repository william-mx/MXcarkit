#!/bin/bash

# set up a hotspot with the name 'mxck0000' and the password 'mxck0000'
sudo nmcli dev wifi hotspot ifname wlan0 ssid mxck0000 password mxck0000

# run docker in new terminal
sleep 5
gnome-terminal --tab -- "/home/mxck/mxck_ws/mxck_base/run_ros_docker.sh" 'true'









