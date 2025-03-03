#!/bin/bash

ifconfig
sudo ifconfig enp88s0 192.168.1.50
sudo ifconfig wlp0s20f3 192.168.1.50
sudo ifconfig enx68da73ac8032 192.168.1.50

cd /home/thesky/RM25_Radar
# . devel/setup.bash
source install/setup.bash

# roscore
#roslaunch livox_ros_driver livox_lidar_rviz.launch
# roslaunch livox_ros_driver livox_lidar.launch
