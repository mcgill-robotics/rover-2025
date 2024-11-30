import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_worlds = get_package_share_directory("sim")

    # Argument to specify the Gazebo world
    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_project_worlds, "model", "worlds", "marsyard2020.sdf"),
        description="Path to the Gazebo world file",
    )

    # Launch the Gazebo simulation with the specified world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    return LaunchDescription(
        [
            sim_world,
            gz_sim,
        ]
    )

