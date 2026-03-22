#!/usr/bin/env python3
"""
rsp.launch.py — Robot State Publisher unificado
================================================
Lanza robot_state_publisher para robot real O simulación.
Consolida: rsp.launch.py (real) + rsp_sim.launch.py (Gazebo básico)

Uso real (default):
  ros2 launch smart_t_ai_v2 rsp.launch.py

Uso simulación Gazebo (plugin nativo):
  ros2 launch smart_t_ai_v2 rsp.launch.py simulation:=true use_sim_time:=true

Nota: Para simulación con ros2_control usar launch_sim_ros2_control.launch.py,
que llama xacro via Command() con use_ros2_control:=true.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    simulation   = LaunchConfiguration('simulation')

    pkg_path   = get_package_share_directory('smart_t_ai_v2')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Procesar URDF — flags comunes a real y sim Gazebo básica
    robot_description_config = xacro.process_file(xacro_file, mappings={
        'use_ros2_control': 'false',
        'sim_mode':         'false',
    })
    robot_desc_xml = robot_description_config.toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_xml,
            'use_sim_time':      use_sim_time,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar tiempo de simulación Gazebo si true'),

        DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='Modo simulación (informativo — afecta logs)'),

        node_robot_state_publisher,
    ])
