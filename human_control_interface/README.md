# human_control_interface Package

## gamepad_input_pub.py

This file creates a ros node which instantiates an instance of the gamepad class from Gamepad.py to listen for controller inputs, and publishes it to the "gamepad_input" topic

To run this node, type this following into the command line: ros2 run human_control_interface gamepad_input_pub

## Gamepad.py

This file uses pygame to connect to controllers, and listens for new controller inputs