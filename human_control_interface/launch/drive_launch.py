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

    pantilt = Node(
        package='drive',
        executable='pantilt_control_node',
        name='pantilit_control',
        output='screen'
    )

    return LaunchDescription([
        drive_control,
        drive_firmware,
        pantilt
    ])

"""
- drive control node
- drive firmware node
- arm control node
- arm firmwware node
- science node
- pantilt node
- gps basestation node (tbd)
- other:
    - jetson backend (rover)
    - webrtc_server (base)
"""
