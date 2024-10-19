from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    # Node for Game Controller
    game_controller = Node(
        package='human_control_interface',
        executable='GamepadProcess',
        name='gamepad',
        output='screen'
    ),

    # Node for Drive Controller
    drive_controller = Node(
        package='drive_control',
        executable='drive_control_node',
        name='drive_controller',
        output='screen'
    ),

    # Set the gamepad to send command to the arm system initially
    ros_topic = Node(
        package='rostopic',
        executable='rostopic',
        name='system_selection',
        ros_arguments=[
            "pub system_selection std_msgs/Int16 'data: 0'",
        ]
    )
    
    return LaunchDescription([game_controller, drive_controller, ros_topic])