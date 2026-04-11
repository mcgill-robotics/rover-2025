from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:

    drive_control = Node(
        package='drive',
        executable='drive_control_node_V2',
        name='drive_control',
        output='screen'
    )

    return LaunchDescription([drive_control])