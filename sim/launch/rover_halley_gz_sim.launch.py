from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_sim = get_package_share_directory("sim")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="empty_world.sdf",
        description="World file to use in Gazebo",
    )

    xacro_file_path = os.path.join(pkg_sim, "halley", "halley_rover.urdf")
    robot_desc = ParameterValue(Command(["xacro ", xacro_file_path]), value_type=str)

    gz_world_arg = PathJoinSubstitution(
        [pkg_sim, "halley", "worlds", world]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_world_arg}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "rover",
            "-allow_renaming", "true",
            "-x", "0",
            "-y", "0",
            "-z", "0.5",  # spawn slightly above ground
        ],
        output="screen",
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odometry/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_desc
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(gz_sim)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)

    return ld