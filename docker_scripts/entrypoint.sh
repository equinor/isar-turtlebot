#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/catkin_ws/devel/setup.bash

export TURTLEBOT3_MODEL=waffle

WORLD_NAME="house"

SCRIPTDIR="$( cd "$( dirname "${BASH_SOURCE}" )" >/dev/null && pwd )"
CONFIG_FILE="$SCRIPTDIR/config/$WORLD_NAME.cfg"

source "$CONFIG_FILE"
if [ ! $? -eq 0 ]
then
  printf "Unable to find config file $CONFIG_FILE\n"
  exit 1
fi

roslaunch isar_turtlebot simulation.launch world_name:=$WORLD_NAME initial_pose_x:=$INITIAL_POSE_X initial_pose_y:=$INITIAL_POSE_Y x_pos:=$X_POS y_pos:=$Y_POS z_pos:=$Z_POS
