import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('my_bot')
    package_name = 'my_bot'
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf -v 4'}.items()
    )

    

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_bot', '-topic', 'robot_description', '-z', '0.01'],
        output='screen'
    )

    # --- Use ExecuteProcess for controller loading ---
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    spawn_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_cont'],
        output='screen'
    )

    # --- Ensure controllers load after the robot is spawned ---
    load_controllers_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                spawn_joint_state_broadcaster,
                spawn_diff_drive_controller,
            ]
        )
    )


    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        load_controllers_event_handler
    ])