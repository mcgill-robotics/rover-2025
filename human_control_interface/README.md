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