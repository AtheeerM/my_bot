import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('my_bot')
    
    # Robot State Publisher - use gazebo control (not ros2_control)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'rsp.launch.py')]), 
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()  # Set to false
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_bot', '-topic', 'robot_description', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])