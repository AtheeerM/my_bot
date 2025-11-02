#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Path to your xacro file
    pkg_share = get_package_share_directory('my_bot')  # change if your package name differs
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    # Build the xacro command (FindExecutable is robust against PATH issues)
    xacro_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file, ' ',
        'use_ros2_control:=', use_ros2_control, ' ',
        'sim_mode:=', use_sim_time
    ])

    # IMPORTANT: ensure robot_description is passed as a STRING, not YAML
    robot_description = ParameterValue(xacro_cmd, value_type=str)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Enable ros2_control interfaces in the xacro'
        ),
        node_robot_state_publisher
    ])
