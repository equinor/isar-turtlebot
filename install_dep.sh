#!/bin/bash

source /opt/ros/noetic/setup.bash

cd /home/catkin_ws/

sudo apt-get update

rosdep update
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

catkin build
