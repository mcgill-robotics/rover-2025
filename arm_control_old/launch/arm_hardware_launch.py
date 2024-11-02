import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='arm_control',
            executable='arm_controller.py',
            name='arm_controller',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='human_control_interface',
            executable='Joystick.py',
            name='joystick',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rosserial_python',
            executable='serial_node.py',
            name='embedded_0'
        ),
        launch_ros.actions.Node(
            package='rosserial_python',
            executable='serial_node.py',
            name='embedded_1'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()