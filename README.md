# isar-turtlebot
ISAR implementation for the Turtlebot3 Waffle Pi.

# Installation for development

For local development, please fork the repository. Then, clone and install in the repository root folder:

```bash
$ git clone https://github.com/equinor/isar-turtlebot
$ cd isar-turtlebot
$ pip install -r requirements.txt -r requirements-dev.txt
$ pip install -e .
```
# Simulation prerequisities

The simulator requires a computer or a virtual machine running Ubuntu. The turtlebot simulator works with different ROS/gazebo distributions, but this installation guide is based on `ROS noetic`. To install and run the turtlebot simulator, you will need to [install ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on your computer. The `desktop` version should be sufficient.


Then, [install dependent ROS packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/), and the [simulation package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).

You also need to install gazebo:
```bash
$ sudo apt-get install gazebo11
```
The installation can be verified by running:
```bash
$ gazebo
```

Install the required [ROS packages](
https://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros) for gazebo:

```bash
$ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

And rosbridge:
```bash
$ sudo apt-get install ros-noetic-rosbridge-suite
```


# Running the simulation

First, you will need to set the following environment variable:

```bash
$ export TURTLEBOT3_MODEL=waffle
```

Build with catkin_make:
```bash
$ cd ~/catkin_ws && catkin_make
```

You will need a map (`map.pgm` and `map.yaml`). The map can be [generated](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation), or the [example map](https://github.com/equinor/isar-turtlebot/tree/main/examples/house) can be used. If you use the example map, you must download the files to your computer and update the image path in `house_map.yaml` with the correct path. Then, gazebo and rviz can be launched:
```bash
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

```bash
$ roslaunch rosbridge_server rosbridge_websocket.launch
```

```bash
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=<path/to/map.yaml>
```

The robot  can be located with `2DPose Estimate` and naviagted with `2D Navigation Goal` in rviz. 

# Running simualtions with ISAR

Download and install [ISAR](github.com/equinor/isar) on your computer. Follow the [robot integration guide](https://github.com/equinor/isar#robot-integration) installing `isar-turtlebot`. Remember to set the robot directory environment variable:

```bash
$ export ROBOT_DIRECTORY=isar_turtlebot
```

Make sure that the [simulation](#running-the-simulation) is running, and run ISAR:

```bash
$ python main.py
```

Missions can now be posted to the robot through [ISAR](https://github.com/equinor/isar#running-a-robot-mission).

If the example map are used, you can try the example mission number `5`.
