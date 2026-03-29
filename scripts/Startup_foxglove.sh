#!/bin/bash

cd ..

source install/setup.bash

ros2 launch radar_bringup foxglove_node.launch.py
