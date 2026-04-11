from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:


    drive_control = Node(
        package='drive',
        executable='drive_control_node',
        name='drive_control',
        output='screen'
    )

    drive_firmware = Node(
        package='drive',
        executable='drive_firmware_node',
        name='drive_firmware',
        output='screen'
    )

    return LaunchDescription([drive_control, drive_firmware])