#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/catkin_ws/devel/setup.bash

export TURTLEBOT3_MODEL=waffle
roslaunch isar_turtlebot simulation.launch
