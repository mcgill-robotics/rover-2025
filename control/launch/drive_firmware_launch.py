from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:

    #Node for Gamepad
    gamepad = Node(
        package='human_control_interface',
        executable='gamepad_input_pub',
        name='gamepad',
        output='screen'
    )

    drive_firmware = Node(
        package='control',
        executable='drive_firmware_node',
        name='drive_firmware',
        output='screen'
    )

    can_communication = Node(
        package='control',
        executable='driveCAN_firmware_node',
        name='can_communication',
        output='screen'
    )

    return LaunchDescription([gamepad, drive_firmware, can_communication])