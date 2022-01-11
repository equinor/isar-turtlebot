#!/bin/bash

source /opt/ros/noetic/setup.bash

cd /home/catkin_ws/

sudo apt-get update

rosdep update
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

catkin build

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source /home/catkin_ws/devel/setup.bash'>> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle'>> ~/.bashrc
