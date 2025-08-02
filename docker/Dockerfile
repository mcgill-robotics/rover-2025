# Use the official ROS Humble base image
FROM ros:humble-ros-core

# Set environment variables for ROS Humble and workspace directory
ENV ROS_DISTRO=humble
ENV WORKSPACE_DIR=/root/rover25_ws

# Update and install system and ROS packages
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git \
    build-essential \
    cmake \
    flex \
    bash \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    libasio-dev \
    libtinyxml2-dev \
    libcurl4-openssl-dev \
    curl \
    iputils-ping \
    iproute2 \
    ros-humble-ament-cmake-python \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    # GStreamer packages
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    v4l-utils && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    vcstool \
    scipy \
    numpy \
    pygame \
    odrive \
    python-can \
    pyserial


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