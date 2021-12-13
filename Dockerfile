FROM osrf/ros:noetic-desktop-full

# nvidia-container-runtime
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
    git


RUN mkdir -p catkin_ws/src

WORKDIR /home/catkin_ws/src/

RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

COPY ./ros_packages/ /home/catkin_ws/src/
COPY ./install_dep.sh /home/
RUN /home/install_dep.sh

COPY ./entrypoint.sh /home/

CMD /home/entrypoint.sh
