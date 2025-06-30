#!/bin/bash
source /opt/ros/humble/setup.bash

gnome-terminal -- bash -c "cd /home/thesky/ws_livox && source install/setup.bash && ros2 launch livox_ros2_driver livox_lidar_rviz_launch.py; exec bash"

gnome-terminal -- bash -c "cd /home/thesky/RM25_Radar && source install/setup.bash && ros2 launch radar_bringup lidar.launch.py; exec bash"
