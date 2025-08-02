from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:


    arm_control = Node(
        package='arm_control',
        executable='arm_control_node',
        name='arm_control',
        output='screen'
    )

    arm_firmware = Node(
        package='arm_control',
        executable='arm_firmware_node',
        name='arm_firmware',
        output='screen'
    )

    return LaunchDescription([arm_control, arm_firmware])