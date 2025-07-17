#!/bin/bash

cd /home/thesky/RM25_Radar

source install/setup.bash

ros2 launch radar_bringup foxglove_node.launch.py
