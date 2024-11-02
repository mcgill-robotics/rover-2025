import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='human_control_interface',
            executable='Joystick.py',
            name='gamepad',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='arm_control',
            executable='arm_controller.py',
            name='arm_controller',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='arm_control',
            executable='arm_sim.py',
            name='arm_sim',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()