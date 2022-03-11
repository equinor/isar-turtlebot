#!/bin/bash

ign launch -v 4 simulation.ign >&1 &

source /opt/ros/noetic/setup.bash
source /home/ws/devel/setup.bash

roscore >&1 &
sleep 2
rosrun ros_ign_bridge parameter_bridge \
    /world/car_world/pose/info@geometry_msgs/PoseArray@ignition.msgs.Pose_V \
    >&1 &

wait -n
exit $?
