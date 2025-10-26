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

    # --- Process URDF ---
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # --- Controller configuration YAML ---
    controller_config = os.path.join(pkg_path, 'config', 'diff_drive_controlles.yaml')

    # --- Nodes ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
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

    # --- ros2_control node (controller manager) ---
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )

    # --- Spawn controllers after robot is loaded ---
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    spawn_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
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
        robot_state_publisher,
        gazebo,
        controller_manager,
        spawn_entity,
        load_controllers_event_handler
    ])
