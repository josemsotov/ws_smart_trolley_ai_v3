#!/usr/bin/env python3
"""
Person Follower — Simulación + Motores reales
==============================================
Lanza 3-4 nodos en paralelo:
  1. fake_depth_publisher  → publica /kinect/depth/image_raw sintético
                             (persona blob controlable con W/S/A/D)
  2. person_follower       → consume el depth, publica /cmd_vel
  3. cmd_vel_dashboard     → muestra las velocidades en tiempo real
  4. arduino_bridge        → (opcional, use_motors:=true) envía /cmd_vel al Arduino

Uso:
  ros2 launch smart_t_ai_v2 person_follower_sim.launch.py               # solo sim
  ros2 launch smart_t_ai_v2 person_follower_sim.launch.py use_motors:=true        # con motores
  ros2 launch smart_t_ai_v2 person_follower_sim.launch.py use_motors:=true serial_port:=/dev/ttyACM0
  ros2 launch smart_t_ai_v2 person_follower_sim.launch.py init_distance:=2.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('smart_t_ai_v2')
    hw_config = os.path.join(pkg, 'config', 'hardware.yaml')

    # ── Args ────────────────────────────────────────────────────────────────
    use_motors      = LaunchConfiguration('use_motors')
    use_kinect      = LaunchConfiguration('use_kinect')
    init_distance   = LaunchConfiguration('init_distance')
    serial_port     = LaunchConfiguration('serial_port')
    target_distance = LaunchConfiguration('target_distance')
    min_distance    = LaunchConfiguration('min_distance')

    return LaunchDescription([
        # ── Declaraciones ──────────────────────────────────────────────────
        DeclareLaunchArgument(
            'use_motors', default_value='false',
            description='true = conectar arduino_bridge a /dev/ttyACM0 (motores reales)'),
        DeclareLaunchArgument(
            'use_kinect', default_value='false',
            description='true = usar Kinect real en vez de fake_depth_publisher'),
        DeclareLaunchArgument(
            'init_distance', default_value='1.5',
            description='Distancia inicial de la persona simulada (metros)'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='Puerto serial del Arduino'),
        DeclareLaunchArgument(
            'target_distance', default_value='1.0',
            description='Distancia objetivo al seguir persona (metros)'),
        DeclareLaunchArgument(
            'min_distance', default_value='0.4',
            description='Distancia mínima de seguridad (metros)'),

        # ── Info ────────────────────────────────────────────────────────────
        LogInfo(msg=''),
        LogInfo(msg='╔══════════════════════════════════════════════════════╗'),
        LogInfo(msg='║   PERSON FOLLOWER — SIMULACIÓN + MOTORES            ║'),
        LogInfo(msg='╠══════════════════════════════════════════════════════╣'),
        LogInfo(msg='║  Ventana 1: Fake Depth (persona blob)               ║'),
        LogInfo(msg='║    W/S = acercar / alejar persona                   ║'),
        LogInfo(msg='║    A/D = mover persona izq / der                    ║'),
        LogInfo(msg='║  Ventana 2: cmd_vel Dashboard (velocidades)         ║'),
        LogInfo(msg='╠══════════════════════════════════════════════════════╣'),
        LogInfo(msg='║  use_motors:=false → /cmd_vel solo al dashboard     ║'),
        LogInfo(msg='║  use_motors:=true  → /cmd_vel también a los motores ║'),
        LogInfo(msg='║  use_kinect:=true  → usa Kinect real (no blob)      ║'),
        LogInfo(msg='╚══════════════════════════════════════════════════════╝'),
        LogInfo(msg=''),

        # ── Nodo 1a: Fake Depth Publisher (use_kinect:=false) ───────────────
        Node(
            package='smart_t_ai_v2',
            executable='fake_depth_publisher',
            name='fake_depth_publisher',
            output='screen',
            parameters=[{
                'depth_topic':   '/kinect/depth/image_raw',
                'publish_rate':  15.0,
                'init_distance': init_distance,
                'init_lateral':  0.0,
                'show_preview':  True,
            }],
            condition=UnlessCondition(use_kinect),
        ),

        # ── Nodo 1b: Kinect real (use_kinect:=true) ──────────────────────────
        Node(
            package='smart_t_ai_v2',
            executable='kinect_node',
            name='kinect_node',
            output='screen',
            parameters=[{
                'device_index':   0,
                'publish_rate':   15.0,
                'rgb_frame_id':   'kinect_rgb_optical',
                'depth_frame_id': 'kinect_depth_optical',
                'publish_rgb':    True,
                'publish_depth':  True,
            }],
            condition=IfCondition(use_kinect),
        ),

        # ── Nodo 2: Person Follower ──────────────────────────────────────────
        Node(
            package='smart_t_ai_v2',
            executable='person_follower',
            name='person_follower',
            output='screen',
            parameters=[{
                'target_distance':    target_distance,
                'min_distance':       min_distance,
                'max_distance':       3.5,
                'max_linear_speed':   0.22,   # conservador para robot real
                'max_angular_speed':  0.6,
                'linear_kp':          0.5,
                'angular_kp':         1.2,
                'depth_threshold':    0.1,
                'roi_width_ratio':    0.65,
                'roi_height_ratio':   0.75,
                'min_detection_area': 400,
                'depth_topic':        '/kinect/depth/image_raw',
                'cmd_vel_topic':      '/cmd_vel',
                'enable_debug':       True,
            }],
        ),

        # ── Nodo 3: cmd_vel Dashboard ────────────────────────────────────────
        Node(
            package='smart_t_ai_v2',
            executable='cmd_vel_dashboard',
            name='cmd_vel_dashboard',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'max_linear':    0.35,
                'max_angular':   1.0,
            }],
        ),

        # ── Nodo 4: Arduino Bridge (solo si use_motors:=true) ────────────────
        Node(
            package='smart_t_ai_v2',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
            parameters=[
                hw_config,
                {'serial_port': serial_port},
            ],
            condition=IfCondition(use_motors),
        ),
    ])

