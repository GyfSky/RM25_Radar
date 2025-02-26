#!/bin/bash

now=$(date +"%Y-%m-%d_%H_%M_%S")
# path="/media/plusseven/KESU/img_DATA/test_dir/${now}.bag"
path="/home/thesky/bags/${now}.bag"
topics="livox/lidar"
all_node_name="__name:=mid70"
cmd_str="gnome-terminal -x bash -c 'ros2 bag record -o ${path} ${topics} ${all_node_name}'"

echo "cmd_str: ${cmd_str}"
echo "path: ${path}"

eval $cmd_str