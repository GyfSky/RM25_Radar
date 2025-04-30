#!/bin/bash
source /opt/ros/humble/setup.bash

gnome-terminal -- bash -c "cd /home/thesky/RM25_Radar && source install/setup.bash && ros2 launch lidar_registration rm24_lidar.launch.py; exec bash"
