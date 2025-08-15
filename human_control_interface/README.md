# human_control_interface Package

## gamepad_input_pub.py

This file creates a ros node which instantiates an instance of the gamepad class from Gamepad.py to listen for controller inputs, and publishes it to the "gamepad_input" topic

To run this node, type this following into the command line after running `colcon build && source install/setup.bash` in your rover25_ws directory: 

```
ros2 run human_control_interface gamepad_input_pub
```

### ROS Topics

| Topic Name            | Direction | Message Type                   | Purpose |
|-----------------------|-----------|--------------------------------|---------|
| `gamepad_input_drive` | Publish   | `msg_srv_interface/msg/GamePadInput` | Publishes the main gamepad input for driving. |
| `gamepad_input_arm`   | Publish   | `msg_srv_interface/msg/GamePadInput` | Publishes gamepad input for controlling the robotic arm. |


## Gamepad.py

This file uses pygame to connect to controllers, and listens for new controller inputs

---

## Launching Nodes

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
ros2 launch human_control_interface drive_launch.py
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