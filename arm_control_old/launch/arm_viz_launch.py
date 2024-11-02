import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=get_package_share_directory(
                'arm_control') + '/model/MR_arm.urdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory(
                'arm_control') + '/rviz/arm_viz.rviz'
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'robot_description': None
                },
                {
                    'use_gui': launch.substitutions.LaunchConfiguration('gui')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'robot_description': None
                },
                {
                    'use_gui': launch.substitutions.LaunchConfiguration('gui')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='odrive_interface',
            executable='node_control_gui.py',
            name='node_control_gui',
            output='screen',
            parameters=[
                {
                    'robot_description': None
                },
                {
                    'use_gui': launch.substitutions.LaunchConfiguration('gui')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='arm_control',
            executable='arm_viz_rviz.py',
            name='arm_viz_rviz',
            output='screen',
            parameters=[
                {
                    'robot_description': None
                },
                {
                    'use_gui': launch.substitutions.LaunchConfiguration('gui')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()