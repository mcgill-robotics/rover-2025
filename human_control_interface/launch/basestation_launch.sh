#!/bin/bash

SESSION_NAME="ros_workspace"

# NOTE: This script assumes you have your rover25 workspace set up at ~/rover25_ws
# and that you have tmux installed.

# Navigate between Tmux windows using:
# - Ctrl+b then 0 for ROS Human Control Interface
# - Ctrl+b then 1 for Camera Backend + Front End UI
# - Ctrl+b then 2 for GPS API + GPS Service

# Start new tmux session with window 3 (ROS) first
tmux new-session -d -s "$SESSION_NAME" -n "ROS"

# Window 3: ROS launch
tmux send-keys -t "$SESSION_NAME":0 "source ~/rover25_ws/install/setup.bash" C-m
tmux send-keys -t "$SESSION_NAME":0 "ros2 launch human_control_interface human_control_interface_launch.py" C-m

# Create window 1: camera_backend and robot_ui split
tmux new-window -t "$SESSION_NAME" -n "Cam+UI"
# Top pane: camera_backend
tmux send-keys -t "$SESSION_NAME":1 "cd ~/rover25_ws/src/teleop/services/camera && ./run_backend.sh" C-m
# Split window vertically
tmux split-window -v -t "$SESSION_NAME":1
# Bottom pane: robot_ui
tmux send-keys -t "$SESSION_NAME":1.1 "cd ~/rover25_ws/src/teleop/robot-controller-ui && npm run dev" C-m

# Create window 2: gps_api and gps_service split
tmux new-window -t "$SESSION_NAME" -n "GPS"
# Top pane: gps_api.py
tmux send-keys -t "$SESSION_NAME":2 "cd ~/rover25_ws/src/teleop/services/gps && python3 gps_api.py" C-m
# Split window vertically
tmux split-window -v -t "$SESSION_NAME":2
# Bottom pane: gps_service.py
tmux send-keys -t "$SESSION_NAME":2.1 "cd ~/rover25_ws/src/teleop/services/gps && python3 gps_service.py" C-m

# Attach to the session, starting on window 3 (ROS)
tmux select-window -t "$SESSION_NAME":0
tmux attach-session -t "$SESSION_NAME"
