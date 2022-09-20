# isar-turtlebot

ISAR implementation for the Turtlebot3 Waffle Pi.

[ISAR](https://github.com/equinor/isar) - Integration and Supervisory control of Autonomous Robots - is a tool for integrating robot applications into Equinor systems. Through the ISAR API you can send command to a robot to do missions and collect results from the missions.

Running the full ISAR system requires an installation of a robot which satisfies the required [interface](https://github.com/equinor/isar/blob/main/src/robot_interface/robot_interface.py). isar-turtlebot is an ISAR implementation of the Turtlebot3 Waffle Pi, which enables running the Turtlebot with ISAR through simulations or physically.

## Run the simulation using docker

### Give Docker containers access to the Nvidia graphics card on the host machine

#### Pre-requisites

- git
- docker
- docker-compose

NOTE: Docker must NOT be installed using Snap. The Snap version is not compatible with nvidia-docker2. Instead, follow the [official documentation](https://docs.docker.com/engine/install/ubuntu/) from Docker for installation.

To check if your Docker version was installed using snap, run the following command

```sh
systemctl list-units --type=service | grep docker
```

If the result is `snap.docker.dockerd.service`, the installation has been done using snap and docker must be reinstalled.

#### Installation nvidia docker

1. Install the newest recommended drivers from Nvidia if not already as described by the following [documentation](https://linuxconfig.org/how-to-install-the-nvidia-drivers-on-ubuntu-20-04-focal-fossa-linux)
2. Install nvidia-docker2 using the [official documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

### Run simulation

Build the container. This needs to be done once before one can give the container access to the screen.

```bash
docker-compose build
```

Give the docker container access to the screen, this needs to be done each time the computer is restarted.

```bash
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' turtle_sim`
```

Start the simulation

```bash
docker-compose up
```

To build and start the simulation

```bash
docker-compose up --build
```

The simulation world that is used can be set by changing the world variable in the 'entrypoint.sh' file.

Additional settings, such as using Nvidia GPU or gamepad input to the docker container is included via separate .yml-files. To run the docker container with these settings the corresponding `docker-compose-setting.yml` file must be specified together with the main `docker-compose.yml` file. Several setting files can be included.

```bash
docker-compose -f docker-compose.yml -f docker-compose-setting.yml up --build
```

To run the simulation in headless mode set the environment variable `HEADLESS=true` prior to launching the docker container. Alternatively directly as an environment variable in the docker-compose command:

```bash
HEADLESS=true docker-compose up --build
```

The simulation can then be viewed at webviz with the following link: https://webviz.io/app/?rosbridge-websocket-url=ws://localhost:9090/

### Video Stream

A video stream of the front camera is found on the address

```bash
http://localhost:5000/stream_viewer?topic=/camera/rgb/image_raw
```

## Adding new models

New models can be added by placing the model that is used by Gazebo into 'models/new_world/' and adding a "new_world.world" file into 'worlds/'. The map that is used by the planner should be placed into 'maps/' with the name 'new_world.\*'. To add a default configuration for the initial pose and position of the robot in the simulation, add 'config/new_world.cfg' with the desired parameters.

### Add new model from turtlesimmodels storage account

Sett the following secret:

```bash
export STORAGE_ACCOUNT_KEY=secret-key-here
```

Then run the following:

```bash
./path-to-repo/az_scripts/download_sim_models.sh
```


### Running custom model

To run the simulation with the custom model set the WORLD_NAME as an environment variable in a .env file or argument like this:

```bash
WORLD_NAME=your_custom_model docker-compose up --build
```
## Simulation without docker

### Simulator installation

The simulator requires a computer or a virtual machine running Ubuntu. The turtlebot simulator works with different ROS/gazebo distributions, but this installation guide is based on `ROS noetic`. To install and run the turtlebot simulator, you will need to [install ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on your computer. The `desktop` version should be sufficient.

Then, [install dependent ROS packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/), and the [simulation package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).

You also need to install gazebo with a version corresponding to your ROS distribution:

```bash
sudo apt-get install gazebo11
```

The installation can be verified by running:

```bash
gazebo
```

Install the required [ROS packages](https://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros) for gazebo:

```bash
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

### Running the simulation

First, you will need to set the following environment variables:

```bash
export ISAR_TURTLEBOT_PATH=<path/to/isar-turtlebot>
export TURTLEBOT3_MODEL=waffle
```

Build with catkin_make:

```bash
cd ~/catkin_ws && catkin_make
```

You will need a map (`map.pgm` and `map.yaml`). The map can be [generated](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation), or the [example map](https://github.com/equinor/isar-turtlebot/tree/main/maps) can be used.

The gazebo together with the navigation stack and rosbridge is then launched by:

```bash
roslaunch launch/simulation.launch
```

The robot can be located with `2DPose Estimate` and naviagted with `2D Navigation Goal` in rviz.

### Simulations with ISAR

Download and install [ISAR](github.com/equinor/isar) on your computer. Follow the [robot integration guide](https://github.com/equinor/isar#robot-integration) installing `isar-turtlebot`. Remember to set the robot directory environment variable:

```bash
export ISAR_ROBOT_PACKAGE=isar_turtlebot
```

For the ISAR API to communicate with the simulator, you will need rosbridge:

```bash
sudo apt-get install ros-noetic-rosbridge-suite
```

You can now [run the simulation](#running-the-simulation) and launch rosbridge:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

To run ISAR:

```bash
python main.py
```

Missions can be posted to the robot through [ISAR](https://github.com/equinor/isar#running-a-robot-mission).

If the example map are used, you can try the example mission number `2`.

## Teleoperation

The turtlebot base can be controlled manually by publishing to the ros topic `/cmd_vel`. Using a keyboard, the ros package [teleop_twist_keyboard](https://wiki.ros.org/teleop_twist_keyboard) translates keyboard inputs to ros messages. Similarly, the ros package [teleop_twist_joy](https://wiki.ros.org/teleop_twist_joy) handles joystick messages.

### Keyboard

With the simulation running, open a new terminal. If running in docker, access the docker container with:

```bash
docker exec -it isar_turtlebot bash
```

Install the teleoperation package (if running in docker this package must be installed every time):

```bash
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

With the teleoperation package installed, enable it with:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Joystick

Teleoperating with a joystick requires an additional package, [joy](https://wiki.ros.org/joy), for reading the joystick input and publishing it to a topic. This package should be compatible with any joystick supported by Linux. After you connected the joystick it should be found as an input decvice `/dev/input/jsX`, where X is the unique id-number of the joystick (default `js0`).

To verify if the joystick is setup correctly, find the unique id-number with

```
ls /dev/input/
```

and run

```
jstest /dev/input/jsX.
```

To enable teleoperation with a joystick while running in docker the joystick must be added as a device in `docker-compose.yaml`. This is done by including the `docker-compose-gamepad.yaml` file as decribed in the docker section. Additionally the environment variable `TELEOP_CONTROLLER` must be specified(currently `"xbox"` is the only supported controller):

```bash
TELEOP_CONTROLLER="xbox" docker-compose -f docker-compose.yml -f docker-compose-gamepad.yml up --build
```

To enable teleoperation while running locally, first install the two packages:

```bash
sudo apt-get install ros-noetic-joy  ros-noetic-teleop-twist-joy
```

Enable teleoperation by opening two new terminals and start up `joy_node` and `teleop_node`:

```bash
rosrun joy joy_node
```

```bash
rosrun teleop_twist_joy teleop_node
```

To control the turtlebot, hold in `enable_button` and use the joystick. See documention for [teleop_twist_joy](https://wiki.ros.org/teleop_twist_joy) for more information.

## Simulation with manipulator

The turtlebot can be equipped with a [manipulator](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation) which also can be included in the simulations. Additional software must be installed for the manipulator simulation.

```bash
cd ~/catkin_ws/src/ &&
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git &&
git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git &&
git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git &&
cd ~/catkin_ws && catkin_make
```

The manipulator can be controlled using rviz or a simpler GUI which is enabled by setting the roslaunch argument
`MANIPULATOR_GUI` to `"rviz"` or `"simple"` respectively. With the latter as default value. There is no constraints for running both controllers simultaneously, but such functionality is not implemented. Running simulation with manipulator can be done by the roslaunch command with the prerequisite of having isar-turtlebot installed as a ros package.

```bash
roslaunch isar-turtlebot turtlebot_manipulator.launch open_manipulator_gui:=true
```

Alternatively the simulation with manipulator can also run in docker by including the parameter `ENABLE_MANIPULATOR` and the controller GUI set according to the description above:

```bash
sudo ENABLE_MANIPULATOR=true MANIPULATOR_GUI="rviz" docker-compose up --build
```

The simulation can also run in docker as described in the section for [docker](#run-simulation).

### Manipulator teleoperation

Teleoperating the manipulator during simulation is enabled with the roslaunch argument `TELEOP-CONTROLLER` specifying the controller (currently `'xbox'` is the only supported controller).

```bash
roslaunch isar_turtlebot turtlebot_manipulator.launch teleop_controller:="xbox"

```

To run simulation in docker with teleoperation of manipulator:

```bash
sudo ENABLE_MANIPULATOR=true MANIPULATOR_GUI="rviz" TELEOP_CONTROLLER="xbox" docker-compose -f docker-compose.yml -f docker-compose-setting.yml  up --build

```

## Development

For local development, please fork the repository. Then, clone and install in the repository root folder:

```bash
git clone https://github.com/equinor/isar-turtlebot
cd isar-turtlebot
pip install -e ".[dev]"
```

## Contributing

We welcome all kinds of contributions, including code, bug reports, issues, feature requests, and documentation. The preferred way of submitting a contribution is to either make an issue on github or by forking the project on github and making a pull requests.

We write our commit messages according to [this guide](https://cbea.ms/git-commit/).
