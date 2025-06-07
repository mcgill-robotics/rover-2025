from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    # Node for Game Controller
    game_controller = Node(
        package='human_control_interface',
        executable='gamepad_input_pub', # Name of executable needs to match node name used console_scripts in setup.py
        name='gamepad_input_pub',
        output='screen'
    )
    
    return LaunchDescription([game_controller])