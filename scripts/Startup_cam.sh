#!/bin/bash
source /opt/ros/humble/setup.bash

cd /home/thesky/RM25_Radar

source install/setup.bash

ros2 launch radar_bringup camera_node.launch.py
