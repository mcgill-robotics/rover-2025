# Use the official ROS Humble base image
FROM ros:humble-ros-core

# Set environment variables for ROS Humble and workspace directory
ENV ROS_DISTRO=humble
ENV WORKSPACE_DIR=/root/rover25_ws

# Update and install ROS core dependencies, including rosdep and pip
RUN apt-get update && \
    apt-get install -y \
    git \
    build-essential \
    cmake \
    flex \
    bash \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip\
    libasio-dev \
    libtinyxml2-dev \
    libcurl4-openssl-dev\
    curl\
    ros-humble-ament-cmake-python\
    iputils-ping\
    iproute2\
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 && \
    apt-get clean

    RUN pip3 install vcstool
    RUN pip3 install scipy
    RUN pip3 install numpy
    # TODO: Needed in GUI node, but very large package, determine if needed later
    # RUN pip3 install PyQt5
    RUN pip3 install pygame
    RUN pip3 install odrive 
    RUN pip3 install python-can

# Source ROS setup script
RUN echo "alias sros='source /opt/ros/humble/setup.bash && [ -f install/setup.bash ] && source install/setup.bash'" >> ~/.bashrc
RUN echo "sros" >> ~/.bashrc
RUN echo "source ~/.bashrc" >> /root/.bash_profile
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

# Create workspace
WORKDIR $WORKSPACE_DIR

RUN colcon build && source install/setup.bash
# Set the default command to bash
CMD ["/bin/bash"]