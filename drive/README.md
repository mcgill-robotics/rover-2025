# Drive

## Overview

This ROS2 package processes all inputs from the `gamepad_input_drive` topic, and converts it into wheel speeds and angles which is used by the corresponding CAN firmware API calls. It also handles controlling the pantilt camera as well.

The main ROS nodes are:

- **drive_control_node**
- **drive_firmware_node**
- **pantilit_control_node**

These can be ran directly using Python (in separate terminals) or using a launch file.

To confirm that this is working, you can check for input by echoing the `gamepad_input_drive` or `drive_speed_input` topics. They should all respond dynamically to your controller input.

# Controller Layout

The control schema relies on a geared speed system where the rover will decelerate to 0 when receiving no input. It has two steering modes: tank and dynamic. Here is the full schema mapped:

Triangle -> switch between standard and tank control schemas

### Standard Schema

X -> accelerate forward

O -> accelerate backward

Square -> acknowledge faults (if the rover ever stops responding, hit this!)

Left Trigger (L2) -> gear down (decrease max speed)

Right Trigger (R2) -> gear up (increase max speed)

Left Shoulder Button (L1) -> tank turn left

Right Shoulder Button (R1) -> tank turn right

Left Analog Stick -> dynamic steer (point wheels in direction stick points)

### Tank Control Schema

Square -> acknowledge faults (if the rover ever stops responding, hit this!)

Left Analog Stick -> control speed of left wheels

Right Analog Stick -> control speed of right wheels

# Gamepad Input Message Format

The `gamepad_input_drive` ROS2 topic publishes the following message type:

| Field             | Type     | Description |
|-------------------|----------|-------------|
| x_button          | int8     | Accelerate forward (Steering schema). |
| o_button          | int8     | Accelerate backward (Steering schema). |
| triangle_button   | int8     | Toggle between Steering and tank control schemas. |
| square_button     | int8     | Acknowledge rover motor faults. |
| l1_button         | int8     | Tank turn left (staSteeringndard schema). |
| r1_button         | int8     | Tank turn right (Steering schema). |
| l2_button         | int8     | Gear down (decrease max speed). |
| r2_button         | int8     | Gear up (increase max speed). |
| select_button     | int8     | Reserved/unused in current mapping. |
| start_button      | int8     | Reserved/unused in current mapping. |
| home_button       | int8     | Reserved/unused in current mapping. |
| l3_button         | int8     | Left stick click (currently unused). |
| r3_button         | int8     | Right stick click (currently unused). |
| l_stick_x         | float32  | Direction to point wheels in horizontal axis (Steering schema). |
| l_stick_y         | float32  | Direction to point wheels in vertical axis (Steering schema). Speed and direction of left side wheels (Tank Schema) |
| l_stick_analog    | float32  | Magnitude of left analog stick movement. |
| r_stick_x         | float32  | Right analog stick horizontal axis (currently unused). |
| r_stick_y         | float32  | Speed and direction of right side wheels (Tank Schema)  |
| r_stick_analog    | float32  | Magnitude of right analog stick movement (currently unused). |
| d_pad_x           | float32  | Pantilit camera yaw control |
| d_pad_y           | float32  | Pantilt camera pitch control |

# Control Scripts

Human drive controls consist of five primary scripts: the two drive-specific ros nodes plus three function library scripts.

## `steering.py`
Provides functions to translate input into an array of wheel instructions to allow the rover to steer. Wheel_orientation_rotation() takes joystick input and interprets it as an array of angles to pass to the steering motors on the wheels. This is the function that takes the wheels and points them in a new direction. Rover_rotation() takes a button input and provides an array of wheel speeds in the form [top right, top left, bottom left, bottom right]. This is the function that causes the rover to turn tank style. Note that this is a longer function, since the direction the wheels are pointing influences which wheels need to run forward and which need to run backward. Also includes the functions for adjusting the speed based on the analog stick input for tank control.

## `speed_control.py` 
Provides a speed_controller class that monitors the wheel acceleration and speed for the rover. To tune the rover speed, adjust the attributes assigned in init. Gears provides a list of dictionaries that control the gears the rover locks to. The current (2025-05-31) physical maximum wheel speed is 3200 rpm. Acceleration_rate, as per its name, determines how quickly the rover speeds up. However, do note that this can also be affected by history_size, since that determines the number of previous speeds that are averaged to smooth acceleration. A bigger history size means a lower acceleration rate, regardless of what the acceleration_rate attribute is. Decceleration_rate determines how quickly the rover stops when the user stops acceleration, while downshift_deceleration_rate determines how quickly we slow down when we gear down. Note that to control __turning speed__, you need to look at drive_control_node.py, __not__ speed_control.py. This file only affects straight-line motion.


## `drive_control_node.py` 
This script defines the **`drive_control_node`** ROS2 node, which processes `gamepad_input_drive` messages and converts them into wheel speed and steering angle commands for the rover. It also manages control modes, gear shifting, and fault acknowledgement. It publishes the speeds to the **drive_speed_input** ros topic and wheel angles to `drive_steering_input`

### ROS Topics

| Topic Name           | Direction  | Message Type                         | Purpose |
|----------------------|------------|---------------------------------------|---------|
| `gamepad_input_drive`| Subscribe  | `msg_srv_interface/msg/GamePadInput`  | Receives controller input for driving. |
| `drive_speed_input`  | Publish    | `std_msgs/msg/Float32MultiArray`      | Sends wheel speed commands. |
| `drive_steering_input`| Publish   | `std_msgs/msg/Float32MultiArray`      | Sends wheel orientation (steering angle) commands. |
| `acknowledge_faults` | Publish    | `std_msgs/msg/Bool`                   | Signals fault acknowledgement to stop blocking movement. |


## `drive_firmware_node.py` 
This node akes the wheel speeds from the **drive_speed_input** ROS topic and converts it to instructions the firmware can understand. It calls the functions in driveCANCommunication.py, which send specific messages to the CAN buses. It handles receiving drive commands, broadcasting them over CAN, and collecting motor diagnostics for (`Voltage`, `Current`, `State`, `Temperature`) for each wheels. Talk to electrical for more details on this.

### ROS Topics

| Topic Name           | Direction  | Message Type                                | Purpose |
|----------------------|------------|----------------------------------------------|---------|
| `drive_speed_input`  | Subscribe  | `std_msgs/msg/Float32MultiArray`             | Incoming wheel speed commands from `drive_control_node`. |
| `drive_steering_input`| Subscribe | `std_msgs/msg/Float32MultiArray`             | Incoming steering commands from `drive_control_node`. |
| `acknowledge_faults` | Subscribe  | `std_msgs/msg/Bool`                          | Clears motor faults when `True`. |
| `drive_motors_info`  | Publish    | `msg_srv_interface/msg/DriveMotorDiagnostic` | Publishes motor voltage, current, state, temperature. |
| `drive_speeds_info`  | Publish    | `std_msgs/msg/Float32MultiArray`             | Publishes measured motor speeds. |

### ROS Services

| Service Name          | Request/Response Type                        | Purpose |
|-----------------------|----------------------------------------------|---------|
| `drive_motors_status` | `msg_srv_interface/srv/DriveMotorStatus`     | Returns the health status of each drive motor. |


## `pantilt_control_node.py`
This script defines the **`pantilt`** ROS2 node, which controls a pan-tilt camera mount and publishes GPS and IMU data from the attached hardware.

### ROS Topics

| Topic Name           | Direction  | Message Type                                | Purpose |
|----------------------|------------|---------------------------------------------|---------|
| `gamepad_input_drive`| Subscribe  | `msg_srv_interface/msg/GamePadInput`        | Receive D-pad input for pan/tilt control. |
| `roverGPSData`       | Publish    | `std_msgs/msg/Float32MultiArray`            | Publishes GPS coordinates from the PanTiltGPS hardware as [# of Satellites, Latitude, Longtitude]. |
| `roverIMUData`       | Publish    | `std_msgs/msg/Float32MultiArray`            | Publishes IMU sensor readings from the PanTiltGPS hardware. |

