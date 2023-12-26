#! /bin/bash

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

catkin_ws=/catkin_ws
data_ws=/datasets/campus-x/outdoor
log_dir=/datasets/campus-x/log

CATKIN_WS=$catkin_ws DATA_PATH=$data_ws LOG_DIR=$log_dir tmuxp load 1014-example.yaml