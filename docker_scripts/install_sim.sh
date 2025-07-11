#!/bin/bash/

curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

apt-get update
apt-get install -y ros-dev-tools gz-harmonic

mkdir -p /home/turtlebot4_ws/src
cd /home/turtlebot4_ws/src

git clone https://github.com/turtlebot/turtlebot4_simulator.git -b jazzy
cd /home/turtlebot4_ws
rosdep install --from-path src -yi

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

source /home/turtlebot4_ws/install/setup.bash

echo 'source /opt/ros/jazzy/setup.bash' >> /home/.bashrc
echo 'source /home/turtlebot4_ws/install/setup.bash'>> /home/.bashrc
