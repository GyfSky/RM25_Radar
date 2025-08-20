#!/bin/bash

gnome-terminal -- bash -c "cd /home/thesky/RM25_Radar && source install/setup.bash && ros2 run lidar_registration lidar_registration_node --ros-args --params-file src/radar_bringup/config/default.yaml ; exec bash"

cd /home/thesky/RM25_Radar

source install/setup.bash

ros2 run rps_radar24 calib_main