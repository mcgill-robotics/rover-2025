# Rover Simulation

ROS 2 simulation package for the Mars rover project. Contains robot models, world environments, launch files, and mock nodes for testing.

## Directory Structure

### `halley/` — Current Rover Model (Active)

This is the primary rover model for **Halley**, the current generation rover. It contains everything needed to spawn and simulate the rover in Gazebo.

- **`halley_rover.urdf`** — Compiled/static URDF for the Halley rover.
- **`meshes/`** — STL mesh files for Halley's physical links: the base chassis (`base_link.STL`), two rocker assemblies (`rocker_A.STL`, `rocker_B.STL`), and four wheels (`wheel_1–4.STL`). The rocker-bogie suspension geometry is reflected in this structure.
- **`worlds/`** — SDF world files for simulation environments:
  - `empty_world.sdf` — Blank environment for basic testing.
  - `depot.sdf` — Indoor depot/warehouse-style environment.
  - `warehouse.sdf` — Larger warehouse layout.
  - `maze.sdf` — Maze environment for navigation testing.

### `model/` — Legacy Rover Model (Galileo, Retired)

This directory contains the robot description for **Galileo**, the previous generation rover. Galileo is **retired and no longer actively developed**, but the model is preserved here for reference and archival purposes. It includes a more complete model with a robotic arm (`MR_arm_*.urdf.xacro`), sensor descriptions (camera, depth camera, IMU, lidar, thermal camera), a full mesh library, and the same set of world SDF files. Do not use this model for new work — refer to `halley/` instead.

### `launch/`

Launch files for bringing up various simulation and visualization configurations:

- `rover_halley_gz_sim.launch.py` — Launch Halley in Gazebo (use this for active development).
- `rover_gz_sim.launch.py` — General Gazebo sim launch (legacy/Galileo).
- `rover_rviz_display.launch.py` — Visualize the rover model in RViz.
- `rover_arm_moveit2.launch.py` — MoveIt 2 bringup for the robotic arm.
- `mock_drive_test.launch.py` — Launch the mock drive firmware node for hardware-free testing.

### `mock_nodes/`

Lightweight mock ROS 2 nodes for testing without physical hardware.

- **`mock_drive_firmware_node.py`** — Simulates the drive firmware interface, allowing drive stack testing in the loop. Meant for testing the front-end UI with ROS
- **`config/test_scenarios.yaml`** — Configurable test scenarios for the mock node.

### `rviz/`

- **`urdf.rviz`** — RViz configuration for URDF/model visualization.

## Getting Started

To launch the Halley rover in Gazebo:

```bash
ros2 launch sim rover_halley_gz_sim.launch.py
```

To run mock drive ROS nodes (no hardware required):

```bash
ros2 launch sim mock_drive_test.launch.py
```