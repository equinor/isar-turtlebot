#!/bin/bash
source /opt/ros/melodic/setup.bash
roscore > /dev/null 2>&1 &

source /opt/ros/melodic/setup.bash
# rosrun ros_ign_brigde parameter_bridge -h /world/car_world/pose/info > /dev/null 2>&1 &

# ign launch -v 4 simulation.ign
