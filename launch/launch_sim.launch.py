import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # === YOUR PACKAGE NAME ===
    package_name = 'my_bot'  # <-- Change to your actual package name if different

    # === PATHS ===
    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'fortress.sdf'
    )

    # === ROBOT STATE PUBLISHER ===
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # === GAZEBO (IGNITION / ROS-GZ) ===
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '--gui', world_file],
        output='screen'
    )

    # === SPAWN ENTITY ===
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',  # modern command for spawning in ros_gz_sim
        arguments=['-topic', 'robot_description', '-name', 'my_bot'],
        output='screen'
    )

    # === RETURN LAUNCH DESCRIPTION ===
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])
