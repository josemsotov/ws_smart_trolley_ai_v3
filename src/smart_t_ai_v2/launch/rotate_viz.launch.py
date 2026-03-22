#!/usr/bin/env python3
"""
rotate_viz.launch.py — Smart Trolley V5
Prueba de rotación 5m → -180° → 5m → +180° con visualización RViz.

Lanza:
  1. robot_state_publisher  (URDF + TF para visualización en RViz)
  2. arduino_bridge.py      (comunicación serial USB con Arduino, publica /odom)
  3. rviz2                  (visualización: modelo robot + trayectoria odometría)
  4. test_5m_rotate.py      (opcional — secuencia automática: 5m, giro, regreso)

Uso:
  # Solo arduino_bridge + RViz (controla con teleop manualmente):
  ros2 launch smart_t_ai_v2 rotate_viz.launch.py run_test:=false

  # Secuencia automática completa:
  ros2 launch smart_t_ai_v2 rotate_viz.launch.py

  # Sin RViz (solo bridge + test):
  ros2 launch smart_t_ai_v2 rotate_viz.launch.py use_rviz:=false

Parámetros:
  serial_port:=/dev/ttyACM0   Puerto Arduino
  use_rviz:=true              Abrir RViz
  run_test:=true              Ejecutar test_5m_rotate automáticamente
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('smart_t_ai_v2')

    xacro_file   = os.path.join(pkg, 'description', 'robot.urdf.xacro')
    arduino_yaml = os.path.join(pkg, 'config', 'arduino_params.yaml')
    rviz_config  = os.path.join(pkg, 'config', 'rviz', 'tests.rviz')

    # ── Argumentos ──────────────────────────────────────────────────────────
    serial_port = LaunchConfiguration('serial_port')
    use_rviz    = LaunchConfiguration('use_rviz')
    run_test    = LaunchConfiguration('run_test')

    args = [
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='Puerto serie del Arduino Mega'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Abrir RViz con visualización'),
        DeclareLaunchArgument(
            'run_test', default_value='true',
            description='Ejecutar test_5m_rotate automáticamente (true/false)'),
    ]

    # ── 1. Robot description (sin ros2_control — arduino_bridge maneja serial) ─
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_ros2_control:=false',
            ' sim_mode:=false',
            ' use_camera:=false',
            ' use_lidar:=false',
            ' use_kinect:=false',
        ]),
        value_type=str
    )

    # ── 2. Robot State Publisher ─────────────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
            'use_sim_time': False,
        }],
    )

    # ── 3. Arduino Bridge (directo, sin ros2_control) ───────────────────────
    arduino_node = Node(
        package='smart_t_ai_v2',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[
            arduino_yaml,
            {
                'serial_port':       serial_port,
                'connection_type':   'usb',
                'encoder_ppr':       45,
                'sensor_profile':    'A',
                'publish_rate':      50.0,
                'encoder_read_rate': 20.0,
            }
        ],
    )

    # ── 4. RViz2 (con delay para que RSP publique el robot_description) ──────
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                condition=IfCondition(use_rviz),
            )
        ]
    )

    # ── 5. test_5m_rotate (con delay para que arduino_bridge conecte) ────────
    #    Test: 5m adelante → giro -180° (CW) → 5m regreso → giro +180° (CCW)
    #    Resultado validado en commit 2a0ea06: ±4.995m, 0° error CW, 3.3° CCW
    rotate_test = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='smart_t_ai_v2',
                executable='test_5m_rotate',
                name='test_5m_rotate',
                output='screen',
                condition=IfCondition(run_test),
            )
        ]
    )

    log = LogInfo(msg=[
        '━━━ Rotate+Viz Test ━━━  ',
        'Puerto: ', serial_port,
        '  |  Secuencia: 5m → -180° → 5m → +180°',
        '  |  RViz: ', use_rviz,
    ])

    return LaunchDescription([
        *args,
        log,
        rsp_node,
        arduino_node,
        rviz_node,
        rotate_test,
    ])
