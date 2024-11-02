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
                'arm_control') + '/rviz/arm_viz_double.rviz'
        ),
        launch_ros.actions.Node(
            package='tf',
            executable='static_transform_publisher',
            name='world_to_robot1'
        ),
        launch_ros.actions.Node(
            package='tf',
            executable='static_transform_publisher',
            name='world_to_robot2'
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz',
            on_exit=launch.actions.Shutdown()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()