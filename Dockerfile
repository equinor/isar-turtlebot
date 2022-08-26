FROM osrf/ros:noetic-desktop-full


ARG TELEOP_CONTROLLER
ENV TELEOP_CONTROLLER=${TELEOP_CONTROLLER}

ARG ENABLE_MANIPULATOR
ENV ENABLE_MANIPULATOR=${ENABLE_MANIPULATOR}

ARG OPEN_MANIPULATOR_GUI
ENV MANIPULATOR_GUI=${MANIPULATOR_GUI}

ARG WORLD_NAME
ENV WORLD_NAME=${WORLD_NAME}


ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


WORKDIR /home/

RUN apt-get update &&\
    apt-get install -y\
    gazebo11 \
    build-essential \
    python3-osrf-pycommon \
    python3-catkin-tools \
    python3-rosdep \
    libignition-rendering3 \
    git \
    xvfb


RUN mkdir -p catkin_ws/src 

WORKDIR /home/catkin_ws/src/

RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git &&\
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git &&\
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git &&\
    git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git


COPY ./ros_packages/ /home/catkin_ws/src/
COPY ./docker_scripts/install_dep.sh /home/
RUN /home/install_dep.sh

RUN mkdir -p /usr/share/gazebo-11/models
COPY ./models /usr/share/gazebo-11/models
COPY ./docker_scripts/setup.sh /usr/share/gazebo-11/setup.sh
COPY ./docker_scripts/setup.sh /usr/share/gazebo/setup.sh

COPY ./config /home/config
COPY ./docker_scripts/entrypoint.sh /home/
COPY ./worlds /home/catkin_ws/src/isar_turtlebot/worlds/
COPY ./maps /home/catkin_ws/src/isar_turtlebot/maps/


# Change user to avoid running as root
# User needs to have an explicit guid for radix
RUN useradd -ms /bin/bash --uid 1001 -g users turtle_sim
RUN chown -R 1001 /home
RUN chmod 755 /home

USER 1001

CMD /home/entrypoint.sh
