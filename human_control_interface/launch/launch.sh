#!/bin/bash

JETSON = "rover@192.168.1.67"
PI = "192.168.1.17"

launch_drive(){
    echo "Connecting to Jetson"
    ssh "$JETSON"
    ros2 launch human_control_interface drive_launch.py
}

launch_arm(){
    echo "Connecting to Pi"  
    ssh "$PI"
    ros2 launch human_control_interface arm_launch.py
}

launch_ui(){
    echo "Launching UI"
    cd human_control_interface/launch/
    ./basestation_launch.sh

    echo -n "Enter device id"
	read -r device_id

    cd ~/rover25_ws/src/teleop/services/camera
    ./run_jetson.sh --device-id "$device_id" 
}

for arg in "$@"; do
    case "$arg" in
        --all)
            launch_drive
            launch_arm
            launch_ui
            break
            ;;
        --drive)
            launch_drive
            ;;
        --arm) 
            launch_arm
            ;;
        --ui)
            launch_ui
            ;;
            *)
            echo "Invalid flag"
            exit 1
            ;;
    esac
done

exit 0