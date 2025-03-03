#!/bin/bash

# 打开新的终端窗口
gnome-terminal
# roscore &

# 打开新的终端窗口
gnome-terminal
cd /home/thesky/RM25_Radar
# . devel/setup.bash
source install/setup.bash
# catkin_make
colcon build
# sh /snap/clion/274/bin/clion.sh  &

echo "StartupFile is finish"