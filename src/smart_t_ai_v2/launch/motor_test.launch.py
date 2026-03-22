#!/usr/bin/env python3
"""
motor_test.launch.py — Smart Trolley V5
Lanza la prueba completa de motores + encoders Hall con RViz.

Arranca:
  1. robot_state_publisher  (URDF para modelo visual)
  2. arduino_bridge         (comunicación con Arduino, odometría)
  3. test_motors_hall       (secuencia de pruebas automatizada)
  4. rviz2                  (visualización: trayectoria + odometría + waypoints)

Uso:
  ros2 launch smart_t_ai_v2 motor_test.launch.py

Opciones:
  ros2 launch smart_t_ai_v2 motor_test.launch.py \\
      test_distance:=1.0 \\
      square_side:=0.8 \\
      linear_speed:=0.2 \\
      run_square:=false \\
      use_rviz:=true \\
      serial_port:=/dev/ttyACM0
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

    # ── Paths ─────────────────────────────────────────────────────────────────
    xacro_file    = os.path.join(pkg, 'description', 'robot.urdf.xacro')
    arduino_yaml  = os.path.join(pkg, 'config',      'arduino_params.yaml')
    rviz_config   = os.path.join(pkg, 'config',      'motor_test.rviz')

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_rviz         = LaunchConfiguration('use_rviz')
    serial_port      = LaunchConfiguration('serial_port')
    sensor_profile   = LaunchConfiguration('sensor_profile')
    encoder_ppr      = LaunchConfiguration('encoder_ppr')
    test_distance    = LaunchConfiguration('test_distance')
    square_side      = LaunchConfiguration('square_side')
    linear_speed     = LaunchConfiguration('linear_speed')
    turn_speed       = LaunchConfiguration('turn_speed')
    run_single_wheel = LaunchConfiguration('run_single_wheel')
    run_square       = LaunchConfiguration('run_square')
    auto_start_delay = LaunchConfiguration('auto_start_delay')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Abrir RViz con configuración de prueba de motores')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Puerto serie del Arduino Mega')

    declare_sensor_profile = DeclareLaunchArgument(
        'sensor_profile', default_value='B',
        description="Perfil de sensor: 'A'=Hall+IMU (45PPR) | 'B'=Hall+Opto (117PPR)")

    declare_encoder_ppr = DeclareLaunchArgument(
        'encoder_ppr', default_value='117',
        description='Pulsos por revolución: 45 (Profile A) | 117 (Profile B Hall 45 + Opto 72)')

    declare_test_distance = DeclareLaunchArgument(
        'test_distance', default_value='0.5',
        description='Distancia (metros) para pruebas de avance/retroceso')

    declare_square_side = DeclareLaunchArgument(
        'square_side', default_value='0.5',
        description='Lado del cuadrado en metros')

    declare_linear_speed = DeclareLaunchArgument(
        'linear_speed', default_value='0.15',
        description='Velocidad lineal de prueba (m/s)')

    declare_turn_speed = DeclareLaunchArgument(
        'turn_speed', default_value='0.4',
        description='Velocidad angular de giro (rad/s)')

    declare_run_single = DeclareLaunchArgument(
        'run_single_wheel', default_value='true',
        description='Ejecutar diagnóstico de ruedas individuales (Hall)')

    declare_run_square = DeclareLaunchArgument(
        'run_square', default_value='true',
        description='Ejecutar patrón cuadrado al final')

    declare_delay = DeclareLaunchArgument(
        'auto_start_delay', default_value='4.0',
        description='Segundos de espera antes de iniciar pruebas')

    # ── Robot description (modo hardware, sin ros2_control) ───────────────────
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

    # ── 1. Robot State Publisher ───────────────────────────────────────────────
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

    # ── 2. Arduino Bridge ─────────────────────────────────────────────────────
    arduino_node = Node(
        package='smart_t_ai_v2',
        executable='arduino_bridge.py',
        name='arduino_bridge',
        output='screen',
        parameters=[
            arduino_yaml,
            {
                'serial_port':      serial_port,
                'sensor_profile':   sensor_profile,
                'encoder_ppr':      encoder_ppr,
                'connection_type':  'usb',
                'publish_rate':     50.0,
                'encoder_read_rate': 10.0,
            }
        ],
        remappings=[],
    )

    # ── 3. Test Motors & Hall (con delay para que arduino_bridge conecte) ──────
    motor_test_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='smart_t_ai_v2',
                executable='test_motors_hall.py',
                name='motor_hall_test',
                output='screen',
                parameters=[{
                    'test_distance':    test_distance,
                    'square_side':      square_side,
                    'linear_speed':     linear_speed,
                    'turn_speed':       turn_speed,
                    'run_single_wheel': run_single_wheel,
                    'run_square':       run_square,
                    'auto_start_delay': auto_start_delay,
                    'wheel_separation': 0.48,
                    'wheel_radius':     0.10,
                }],
            )
        ]
    )

    # ── 4. RViz2 (con delay para que RobotModel cargue) ───────────────────────
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

    # ── Log de inicio ──────────────────────────────────────────────────────────
    log_start = LogInfo(
        msg=['━━━ Motor Test Launch ━━━  ',
             'Puerto: ', serial_port, '  |  ',
             'Distancia: ', test_distance, 'm  |  ',
             'Cuadrado: ', square_side, 'm']
    )

    return LaunchDescription([
        # Argumentos
        declare_use_rviz,
        declare_serial_port,
        declare_sensor_profile,
        declare_encoder_ppr,
        declare_test_distance,
        declare_square_side,
        declare_linear_speed,
        declare_turn_speed,
        declare_run_single,
        declare_run_square,
        declare_delay,
        # Nodos
        log_start,
        rsp_node,
        arduino_node,
        motor_test_node,
        rviz_node,
    ])
