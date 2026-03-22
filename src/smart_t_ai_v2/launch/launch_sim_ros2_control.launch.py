#!/usr/bin/env python3
"""
Launch file para simulación con ros2_control
Usa diff_drive_controller en lugar del plugin nativo de Gazebo
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Package name
    package_name = 'smart_t_ai_v2'
    pkg_path = get_package_share_directory(package_name)

    # Paths
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    controller_config = os.path.join(pkg_path, 'config', 'controllers.yaml')

    # Robot description with sim_mode=true (GazeboSimSystem plugin)
    robot_description = Command([
        'xacro ', xacro_file,
        ' sim_mode:=true'
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Set Gazebo resource paths
    gazebo_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_path, 'worlds')
    )

    # Gazebo Sim launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'smart_trolley',
            '-z', '0.1'
        ],
        output='screen'
    )

    # NOTA: No lanzamos controller_manager manualmente
    # Gazebo lo lanza automáticamente con el plugin gz_ros2_control

    # Delay para esperar que Gazebo inicie el controller_manager
    delay_joint_state_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Diff Drive Controller Spawner (después del joint state broadcaster)
    delay_diff_drive_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Bridge for laser scan
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    # Bridge for camera
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        delay_joint_state_spawner,
        delay_diff_drive_spawner,
        bridge_scan,
        bridge_camera,
    ])
