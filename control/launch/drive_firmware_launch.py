from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:


    drive_firmware = Node(
        package='control',
        executable='drive_control_node',
        name='drive_control',
        output='screen'
    )

    can_communication = Node(
        package='control',
        executable='driveCAN_firmware_node',
        name='can_communication',
        output='screen'
    )

    return LaunchDescription([drive_firmware, can_communication])