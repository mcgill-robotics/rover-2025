from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:


    drive_control = Node(
        package='control',
        executable='drive_control_node',
        name='drive_control',
        output='screen'
    )

    drive_firmware = Node(
        package='control',
        executable='drive_firmware_node',
        name='drive_firmware',
        output='screen'
    )


    arm_control = Node(
        package='arm_control',
        executable='arm_control_node',
        name='arm_control_node',
        output='screen'
    )

    arm_firmware = Node(
        package='arm_control',
        executable='arm_firmware_node',
        name='arm_firmware_node',
        output='screen'
    )

    pantilt = Node(
        package='control',
        executable='pantilt_control_node',
        name='pantilit_control',
        output='screen'
    )

    # TODO
    # antenna_basestation = Node(
    #     package='control',
    #     executable='pantilt_control_node',
    #     name='pantilit_control',
    #     output='screen'
    # )


    return LaunchDescription([drive_control, drive_firmware, arm_control, arm_firmware, pantilt])

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