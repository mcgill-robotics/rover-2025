# rover-2025

## UI
sudo apt install ros-humble-rosbridge-server
colcon build --packages-select control (or colcon build)
source install/setup.bash
source /opt/ros/humble/setup.bash

***ROS2***
1. ros2 launch control control_launch.py

***Bridge***
1. ros2 run rosbridge_server rosbridge_websocket

***UI***
1. npm install
2. npm run start:all