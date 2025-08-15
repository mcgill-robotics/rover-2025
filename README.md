# Rover 2025 Codebase

## Overview

Hello!

Welcome to the McGill Robotics Rover 2025 Repo:)

## Requirements

As we use ROS2 are the framework for our codebase, it requires running a Ubuntu 22.04 environment. To use this repository, we recommend following our guide on the rover2025 wiki on [how to setup your environment](https://github.com/mcgill-robotics/rover-2025/wiki/Softie-102:-Setup-Rover-ROS-2-Humble-Environment).

Another option is building a local Docker Image using our DockerFile, which you can do by following this guide: [Docker Setup](docker/README.md). Note, you will need to edit the path to mount your ROS2 `rover25_ws` workspace to the docker container in the `docker-compose.yml`

## Overview


### `arm_control/`
Contains the ROS2 package for for controlling a 5 DoF robotic arm, using a gamepad (PS4 controller), either by joint-based control or inverse kinematics. Includes:
- **src/**: Arm control ROS nodes, inverse kinematics, CAN firmware scripts.
- **arm_sim/**: Arm simulation scripts.
- **model/**: URDF and mesh files for the arm model.
- **test/**: Unit and integration tests for arm control.
- **archive/**: Legacy or backup scripts.

### `drive/`
ROS2 package for rover drive and pan-tilt control. Drive control encompasses tank and steering based controls. Includes:
- **scripts/**: Drive control nodes, firmware, CAN communication, and pan-tilt modules.
- **test/**: Drive system tests.
- **archive/**: Legacy drive scripts.
- **launch/**: Launch files for drive nodes.

### `human_control_interface/`
ROS2 package for interfacing with human input devices (e.g., gamepads). Includes:
- **src/**: Gamepad input publisher and gamepad interface logic.
- **launch/**: Launch files for human control nodes.
- **resource/**: Package resource files.

### `msg_srv_interface/`
Contains custom ROS2 message and service definitions for inter-package communication.

### `sim/`
ROS2 package for simulation of drive and arm systems.

### `teleop/`
Contains UI and backend services for teleoperation:
- **robot-controller-ui/**: Next.js frontend for rover control and monitoring.
- **services/**: Backend services (e.g., camera, GPS) for teleop system.

### `docker/`
Docker setup for building and running the workspace in a containerized environment. Needed to run ROS2 on the Raspberry Pi 5. Includes Dockerfile, compose files, and setup scripts.

---

## Launching Drive, Arm, and UI

To start multiple nodes at once, use the provided launch files in the `human_control_interface/launch/` directory.

### How to Launch

**Build the workspace (if not already built):**
   ```
   # In ~/rover25_ws directory
   colcon build
   source install/setup.bash
  ```

### Launch Drive Nodes (Jetson)

```
ros2 launch human_control_interface arm_launch.py
```

### Launch Arm Nodes (Pi)

```
ros2 launch human_control_interface arm_launch.py
```

### Launch Basestation UI Backend + Gamepad Input Publisher Node
```
# Assuming you are in the src/ directory
cd human_control_interface/launch/
./basestation_launch.sh
```

### Launch Jetson and Pi Backend for camera UI

```
# You have to run this for both Jetson and Pi
cd ~/rover25_ws/src/teleop/services/camera
./run_jetson.sh --device-id <jetson/pi>-01 # replace <jetson/pi> with the device you are running the backend on, ex. jetson-01
```