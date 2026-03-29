#!/bin/bash
source /opt/ros/humble/setup.bash


gnome-terminal -- bash -c "cd .. && colcon build --parallel-workers 1"
