#!/usr/bin/env python3
"""
Launch file for mock drive firmware node testing.

This launch file allows easy testing of the drive system without hardware
by starting the mock firmware node with configurable scenarios.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for mock drive testing."""
    
    # Declare launch arguments
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='normal',
        description='Test scenario to run (normal, low_battery, motor_fault, etc.)'
    )
    
    diagnostics_rate_arg = DeclareLaunchArgument(
        'diagnostics_rate',
        default_value='10.0',
        description='Rate for publishing motor diagnostics (Hz)'
    )
    
    speeds_rate_arg = DeclareLaunchArgument(
        'speeds_rate',
        default_value='20.0',
        description='Rate for publishing motor speeds (Hz)'
    )
    
    enable_faults_arg = DeclareLaunchArgument(
        'enable_faults',
        default_value='false',
        description='Enable random fault injection for testing'
    )
    
    fault_probability_arg = DeclareLaunchArgument(
        'fault_probability',
        default_value='0.01',
        description='Probability of random fault injection (0.0-1.0)'
    )
    
    realistic_physics_arg = DeclareLaunchArgument(
        'realistic_physics',
        default_value='true',
        description='Enable realistic motor physics simulation'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to custom configuration file (optional)'
    )
    
    # Get the path to the config file
    config_file_path = PathJoinSubstitution([
        FindPackageShare('sim'),
        'mock_nodes',
        'config',
        'test_scenarios.yaml'
    ])
    
    # Mock drive firmware node
    mock_drive_node = Node(
        package='sim',
        executable='mock_drive_firmware_node.py',
        name='mock_drive_firmware_node',
        parameters=[{
            'scenario': LaunchConfiguration('scenario'),
            'diagnostics_rate': LaunchConfiguration('diagnostics_rate'),
            'speeds_rate': LaunchConfiguration('speeds_rate'),
            'enable_faults': LaunchConfiguration('enable_faults'),
            'fault_probability': LaunchConfiguration('fault_probability'),
            'realistic_physics': LaunchConfiguration('realistic_physics'),
            'config_file': LaunchConfiguration('config_file'),
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # Log info about the scenario being launched
    log_scenario = LogInfo(
        msg=['Starting mock drive firmware with scenario: ', LaunchConfiguration('scenario')]
    )
    
    return LaunchDescription([
        scenario_arg,
        diagnostics_rate_arg,
        speeds_rate_arg,
        enable_faults_arg,
        fault_probability_arg,
        realistic_physics_arg,
        config_file_arg,
        log_scenario,
        mock_drive_node,
    ])
