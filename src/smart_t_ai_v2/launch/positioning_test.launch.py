#!/usr/bin/env python3
"""
positioning_test.launch.py — Smart Trolley V5
Lanza la prueba de precisión de posicionamiento.

Arranca:
  1. robot_state_publisher  (URDF para modelo visual)
  2. ros2_control_node      (Controller Manager — interfaz con Arduino)
  3. joint_state_broadcaster spawner (3 s)
  4. diff_drive_controller  spawner  (4.5 s)
  5. test_positioning.py    (6 s — espera que los controladores estén listos)
  6. rviz2 (opcional)

Uso:
  ros2 launch smart_t_ai_v2 positioning_test.launch.py

Parámetros:
  serial_port:=/dev/ttyACM0   Puerto Arduino
  test_distance:=1.0          Distancia de prueba (m)
  linear_speed:=0.20          Velocidad lineal (m/s)
  angular_speed:=0.50         Velocidad angular (rad/s)
  run_return:=true            Incluir prueba de vuelta
  run_square:=true            Incluir prueba de cuadrado
  use_gps:=false              Registrar waypoints GPS
  use_rviz:=true              Abrir RViz
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

    xacro_file      = os.path.join(pkg, 'description', 'robot.urdf.xacro')
    controllers_cfg = os.path.join(pkg, 'config',      'controllers.yaml')
    rviz_config     = os.path.join(pkg, 'config',      'rviz', 'tests.rviz')

    # ── Argumentos ────────────────────────────────────────────────────────────
    serial_port    = LaunchConfiguration('serial_port')
    test_distance  = LaunchConfiguration('test_distance')
    linear_speed   = LaunchConfiguration('linear_speed')
    angular_speed  = LaunchConfiguration('angular_speed')
    run_return     = LaunchConfiguration('run_return')
    run_square     = LaunchConfiguration('run_square')
    use_gps        = LaunchConfiguration('use_gps')
    use_rviz       = LaunchConfiguration('use_rviz')

    args = [
        DeclareLaunchArgument('serial_port',   default_value='/dev/ttyACM0',
            description='Puerto serie del Arduino Mega'),
        DeclareLaunchArgument('test_distance', default_value='1.0',
            description='Distancia de prueba en metros'),
        DeclareLaunchArgument('linear_speed',  default_value='0.20',
            description='Velocidad lineal de avance (m/s)'),
        DeclareLaunchArgument('angular_speed', default_value='0.50',
            description='Velocidad angular de giro (rad/s)'),
        DeclareLaunchArgument('run_return',    default_value='true',
            description='Ejecutar prueba de viaje de vuelta'),
        DeclareLaunchArgument('run_square',    default_value='true',
            description='Ejecutar prueba de cuadrado'),
        DeclareLaunchArgument('use_gps',       default_value='false',
            description='Registrar waypoints GPS si está disponible'),
        DeclareLaunchArgument('use_rviz',      default_value='true',
            description='Abrir RViz con visualización de trayectoria'),
    ]

    # ── Robot description ──────────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' sim_mode:=false',
            ' serial_port:=', serial_port,
            ' use_camera:=false',
            ' use_lidar:=false',
            ' use_kinect:=false',
        ]),
        value_type=str
    )

    # ── Nodos ─────────────────────────────────────────────────────────────────

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

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_cfg,
        ],
    )

    joint_state_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    diff_drive_spawner = TimerAction(
        period=4.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        )]
    )

    positioning_test = TimerAction(
        period=6.0,
        actions=[Node(
            package='smart_t_ai_v2',
            executable='test_positioning',
            name='positioning_test',
            output='screen',
            parameters=[{
                'test_distance': test_distance,
                'linear_speed':  linear_speed,
                'angular_speed': angular_speed,
                'run_return':    run_return,
                'run_square':    run_square,
                'use_gps':       use_gps,
                'auto_delay':    2.0,
            }],
        )]
    )

    rviz_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        )]
    )

    cmd_vel_relay = Node(
        package='smart_t_ai_v2',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
    )

    log = LogInfo(msg=[
        '━━━ Positioning Test ━━━  ',
        'Puerto: ', serial_port,
        '  |  Distancia: ', test_distance, 'm',
        '  |  Vel: ', linear_speed, 'm/s',
    ])

    return LaunchDescription([
        *args,
        log,
        rsp_node,
        ros2_control_node,
        joint_state_spawner,
        diff_drive_spawner,
        cmd_vel_relay,
        rviz_node,
        positioning_test,
    ])
