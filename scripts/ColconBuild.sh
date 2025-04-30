#!/bin/bash
source /opt/ros/humble/setup.bash


gnome-terminal -- bash -c "cd /home/thesky/ws_livox && colcon build && cd /home/thesky/RM25_Radar && colcon build --parallel-workers 1"
