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

    pantilt = Node(
        package='control',
        executable='pantilt_control_node',
        name='pantilit_control',
        output='screen'
    )

    science = Node(
        package='science',
        executable='sensor_publisher',
        name='science_sensor',
        output='screen'
    )

    # TODO
    # antenna_basestation = Node(
    #     package='control',
    #     executable='pantilt_control_node',
    #     name='pantilit_control',
    #     output='screen'
    # )

    return LaunchDescription([drive_control, drive_firmware, pantilt, science])


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