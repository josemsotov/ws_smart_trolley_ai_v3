#!/usr/bin/env python3
"""
simple_motors.launch.py — Smart Trolley V5
Lanza arduino_bridge + prueba/diagnóstico de motores.

Uso:
  ros2 launch smart_t_ai_v2 simple_motors.launch.py                   # avance/retroceso
  ros2 launch smart_t_ai_v2 simple_motors.launch.py diag:=true        # diagnóstico individual
  ros2 launch smart_t_ai_v2 simple_motors.launch.py serial_port:=/dev/ttyUSB0
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('smart_t_ai_v2')
    arduino_yaml = os.path.join(pkg, 'config', 'arduino_params.yaml')

    serial_port = LaunchConfiguration('serial_port')
    diag        = LaunchConfiguration('diag')

    arduino_node = Node(
        package='smart_t_ai_v2', executable='arduino_bridge', name='arduino_bridge',
        output='screen',
        parameters=[arduino_yaml, {'serial_port': serial_port, 'connection_type': 'usb'}])

    # Modo simple: avance 2s → pausa → retroceso 2s
    simple_node = TimerAction(period=3.0, actions=[Node(
        package='smart_t_ai_v2', executable='motor_simple_test', name='simple_motor_test',
        output='screen', condition=UnlessCondition(diag))])

    # Modo diagnóstico: giros individuales + ramp de velocidad
    diag_node = TimerAction(period=3.0, actions=[Node(
        package='smart_t_ai_v2', executable='motor_diag', name='motor_diag',
        output='screen', condition=IfCondition(diag))])

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Puerto serie del Arduino'),
        DeclareLaunchArgument('diag', default_value='false',
                              description='false=simple avance/retroceso | true=diagnóstico individual'),
        arduino_node,
        simple_node,
        diag_node,
    ])
