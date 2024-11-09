# Use the official ROS Humble base image
FROM ros:humble-ros-core

# Set environment variables for ROS and micro-ROS
ENV ROS_DISTRO=humble
ENV WORKSPACE_DIR=/root/microros_ws

# Update and install core dependencies, including rosdep and pip
RUN apt-get update

RUN apt-get install -y \
    git \
    build-essential \
    cmake \
    flex

RUN apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip

RUN apt-get install -y \
    libasio-dev \
    libtinyxml2-dev \
    libcurl4-openssl-dev

RUN apt-get install -y curl
RUN pip3 install vcstool
RUN apt-get install -y ros-humble-ament-cmake-python
RUN sudo apt-get install -y iputils-ping iproute2
RUN apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp

# Source ROS setup script
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

# Create workspace and clone micro-ROS setup repository
RUN mkdir -p $WORKSPACE_DIR/src
WORKDIR $WORKSPACE_DIR/src
RUN if [ ! -d "/root/microros_ws/src/micro_ros_setup" ]; then \
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git /root/microros_ws/src/micro_ros_setup;\
    fi

# Install micro-ROS dependencies
# WORKDIR $WORKSPACE_DIR
# RUN rosdep update
# RUN rosdep install --from-paths src --ignore-src -y
# RUN colcon build
# RUN source install/local_setup.bash
# RUN echo "source $WORKSPACE_DIR/install/local_setup.bash" >> ~/.bashrc

# Set the default command to bash
CMD ["/bin/bash"]