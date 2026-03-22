#!/usr/bin/env python3
"""
Launch: kinect_view.launch.py
──────────────────────────────
Arranca el Kinect y abre RViz2 para ver el stream de cámara en tiempo real.

Opciones:
  with_hailo:=true   También lanza hailo_face_gesture_node (caras + gestos)
  device_index:=0    Índice del dispositivo Kinect
  publish_rate:=15.0 Hz de captura

Uso:
  ros2 launch smart_t_ai_v2 kinect_view.launch.py
  ros2 launch smart_t_ai_v2 kinect_view.launch.py with_hailo:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('smart_t_ai_v2')
    rviz_config = os.path.join(pkg, 'config', 'rviz', 'kinect_view.rviz')

    # ─── Argumentos ───────────────────────────────────────────────────────────
    with_hailo   = LaunchConfiguration('with_hailo')
    device_index = LaunchConfiguration('device_index')
    rate         = LaunchConfiguration('publish_rate')

    declare_with_hailo = DeclareLaunchArgument(
        'with_hailo', default_value='false',
        description='Lanzar también hailo_face_gesture_node (detección de caras)')

    declare_device = DeclareLaunchArgument(
        'device_index', default_value='0',
        description='Índice del dispositivo Kinect (0 = primero)')

    declare_rate = DeclareLaunchArgument(
        'publish_rate', default_value='15.0',
        description='Frecuencia de captura en Hz')

    # ─── Static TF mínimo (map→base_link) para que RViz no se queje ─────────
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='log',
    )

    # ─── Kinect Node ─────────────────────────────────────────────────────────
    kinect_node = Node(
        package='smart_t_ai_v2',
        executable='kinect_node',
        name='kinect_node',
        output='screen',
        parameters=[{
            'device_index':  device_index,
            'publish_rate':  rate,
            'publish_rgb':   True,
            'publish_depth': True,
        }],
    )

    # ─── Hailo Face+Gesture Node (opcional) ──────────────────────────────────
    hailo_node = Node(
        package='smart_t_ai_v2',
        executable='hailo_face_gesture_node',
        name='hailo_face_gesture',
        output='screen',
        condition=IfCondition(with_hailo),
    )

    # ─── RViz2 ───────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        LogInfo(msg='╔══════════════════════════════════════════════════╗'),
        LogInfo(msg='║        KINECT VIEW — Cámara en RViz2             ║'),
        LogInfo(msg='╠══════════════════════════════════════════════════╣'),
        LogInfo(msg='║  Topics publicados:                              ║'),
        LogInfo(msg='║    /kinect/rgb/image_raw   (640×480 RGB)         ║'),
        LogInfo(msg='║    /kinect/depth/image_raw (640×480 Depth)       ║'),
        LogInfo(msg='║  Usa with_hailo:=true para detección de caras    ║'),
        LogInfo(msg='╚══════════════════════════════════════════════════╝'),

        declare_with_hailo,
        declare_device,
        declare_rate,

        static_tf,
        kinect_node,
        hailo_node,
        rviz_node,
    ])
