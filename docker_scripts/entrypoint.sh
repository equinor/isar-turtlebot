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

if [ "$ENABLE_MANIPULATOR" = true ]
then
  if [ "$HEADLESS" = true ]
  then
    xvfb-run --auto-servernum --server-num=1 roslaunch isar_turtlebot turtlebot_manipulator.launch -v world_name:=$WORLD_NAME teleop_controller:=$TELEOP_CONTROLLER initial_pose_x:=$INITIAL_POSE_X initial_pose_y:=$INITIAL_POSE_Y x_pos:=$X_POS y_pos:=$Y_POS z_pos:=$Z_POS manipulator_gui:=$MANIPULATOR_GUI gui:=false headless:=true open_rviz:=false extra_gazebo_args:="--verbose"
  else
    roslaunch isar_turtlebot turtlebot_manipulator.launch -v world_name:=$WORLD_NAME teleop_controller:=$TELEOP_CONTROLLER initial_pose_x:=$INITIAL_POSE_X initial_pose_y:=$INITIAL_POSE_Y x_pos:=$X_POS y_pos:=$Y_POS z_pos:=$Z_POS manipulator_gui:=$MANIPULATOR_GUI extra_gazebo_args:="--verbose"
  fi

else
  if [ "$HEADLESS" = true ]
  then
    xvfb-run --auto-servernum --server-num=1 roslaunch isar_turtlebot simulation.launch -v world_name:=$WORLD_NAME teleop_controller:=$TELEOP_CONTROLLER initial_pose_x:=$INITIAL_POSE_X initial_pose_y:=$INITIAL_POSE_Y x_pos:=$X_POS y_pos:=$Y_POS z_pos:=$Z_POS gui:=false headless:=true open_rviz:=false extra_gazebo_args:="--verbose"
  else
    roslaunch isar_turtlebot simulation.launch -v world_name:=$WORLD_NAME teleop_controller:=$TELEOP_CONTROLLER initial_pose_x:=$INITIAL_POSE_X initial_pose_y:=$INITIAL_POSE_Y x_pos:=$X_POS y_pos:=$Y_POS z_pos:=$Z_POS extra_gazebo_args:="--verbose"
  fi
fi
