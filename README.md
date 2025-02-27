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

***Python virtual environment***
1. python -m venv rosEnv
2. source rosEnv/bin/activate

***ROS2 test for terminal UI***
From rover-2025
1. colcon build --select-packages cpp_pubsub (From Steve's repo)
2. source install/set.bash
3. python3 controller_monitor.py (Make sure to be in venv)