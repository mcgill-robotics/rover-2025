from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Process xacro or urdf for robot_description
    xacro_file_path = os.path.join(
        get_package_share_directory("rover_system"),
        "model",
        # "two_wheel_robot.xacro",
        "rover.urdf.xacro",
        # "MR_arm.urdf",
    )

    # Method 1
    # robot_description_config = xacro.process_file(xacro_file_path)
    # robot_desc = robot_description_config.toxml()
    # Method 2
    robot_desc = ParameterValue(Command(["xacro ", xacro_file_path]), value_type=str)
    # robot_desc = ParameterValue(Command(["xacro ", urdf_file_path]), value_type=str)

    # Start Ignition Gazebo with an empty world
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # Gazebo simulation
    world = LaunchConfiguration("world")
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="depot.sdf",
        description="World file to use in Gazebo",
    )
    gz_world_arg = PathJoinSubstitution(
        [get_package_share_directory("rover_system"), "model", "worlds", world]
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_world_arg}.items(),
    )

    # Spawn the URDF robot model into Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "rover",
            "-allow_renaming",
            "true",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
        ],
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
        ],
    )

    # Start the robot_state_publisher
    params = {"use_sim_time": use_sim_time, "robot_description": robot_desc}
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[params],
        arguments=[],
    )

    # Start RViz
    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("rover_system"),
                "rviz",
                "urdf.rviz",
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)

    # Robot state publisher
    ld.add_action(start_robot_state_publisher_cmd)

    # RViz
    # ld.add_action(rviz_cmd)

    return ld
