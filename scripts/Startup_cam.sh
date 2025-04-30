#!/bin/bash
source /opt/ros/humble/setup.bash

cd /home/thesky/RM25_Radar

source install/setup.bash

ros2 run rps_radar24 RadarMain
