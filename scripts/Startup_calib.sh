#!/bin/bash

gnome-terminal -- bash -c "cd .. && source install/setup.bash && ros2 run lidar_registration lidar_registration_node --ros-args --params-file src/radar_bringup/config/default.yaml ; exec bash"

gnome-terminal -- bash -c "cd .. && source install/setup.bash && ros2 launch livox_ros2_driver livox_lidar_rviz_launch.py; exec bash"

cd ..

source install/setup.bash

ros2 run camera_detector calib_main