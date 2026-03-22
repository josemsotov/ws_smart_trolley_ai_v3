#!/usr/bin/env python3
"""
Launch file para Person Follower con Kinect.
Lanza los sensores necesarios + el nodo de seguimiento.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch completo para person follower:
    - Robot State Publisher
    - Kinect
    - Person Follower
    - RViz (opcional)
    - Arduino Bridge (opcional para ejecutar comandos)
    
    Uso:
      ros2 launch smart_t_ai_v2 person_follower.launch.py
      
    Con Arduino (robot real):
      ros2 launch smart_t_ai_v2 person_follower.launch.py use_arduino:=true
    
    Sin RViz:
      ros2 launch smart_t_ai_v2 person_follower.launch.py use_rviz:=false
    """
    pkg_smart_t = get_package_share_directory('smart_t_ai_v2')

    # ─── Paths ───
    xacro_file = os.path.join(pkg_smart_t, 'description', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_smart_t, 'config', 'rviz', 'sensors.rviz')
    follower_config = os.path.join(pkg_smart_t, 'config', 'navigation.yaml')
    arduino_config = os.path.join(pkg_smart_t, 'config', 'hardware.yaml')

    # ─── Launch Arguments ───
    use_rviz = LaunchConfiguration('use_rviz')
    use_arduino = LaunchConfiguration('use_arduino')
    serial_port = LaunchConfiguration('serial_port')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz for visualization')

    declare_use_arduino = DeclareLaunchArgument(
        'use_arduino', default_value='false',
        description='Launch Arduino Bridge for real robot control')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Arduino serial port')

    # ─── Robot Description ───
    robot_description = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=false',
        ' sim_mode:=false',
        ' use_camera:=false',
        ' use_lidar:=false',
        ' use_kinect:=true',
    ])

    # ─── Robot State Publisher ───
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'publish_frequency': 10.0,
        }]
    )

    # ─── Static TF (sin odometría) ───
    static_tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )

    # ─── Kinect Node ───
    kinect_node = Node(
        package='smart_t_ai_v2',
        executable='kinect_node',
        name='kinect_node',
        output='screen',
        parameters=[{
            'device_index': 0,
            'publish_rate': 15.0,
            'rgb_frame_id': 'kinect_rgb_optical',
            'depth_frame_id': 'kinect_depth_optical',
            'publish_rgb': True,
            'publish_depth': True,
        }],
    )

    # ─── Person Follower ───
    person_follower = Node(
        package='smart_t_ai_v2',
        executable='person_follower',
        name='person_follower',
        output='screen',
        parameters=[follower_config],
    )

    # ─── Arduino Bridge (condicional) ───
    arduino_bridge = Node(
        package='smart_t_ai_v2',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[
            arduino_config,
            {'serial_port': serial_port},
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
        ],
        condition=IfCondition(use_arduino),
    )

    # ─── RViz ───
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        LogInfo(msg='══════════════════════════════════════════════════════'),
        LogInfo(msg='  PERSON FOLLOWER - Kinect-Based Person Tracking'),
        LogInfo(msg='══════════════════════════════════════════════════════'),
        LogInfo(msg='  El robot te seguirá mientras caminas de espalda'),
        LogInfo(msg='  Asegúrate de estar en el campo de visión del Kinect'),
        LogInfo(msg='══════════════════════════════════════════════════════'),
        
        # Arguments
        declare_use_rviz,
        declare_use_arduino,
        declare_serial_port,
        
        # Nodes
        robot_state_publisher,
        static_tf_map_to_base,
        kinect_node,
        person_follower,
        arduino_bridge,
        rviz_node,
        
        LogInfo(msg='──────────────────────────────────────────────────────'),
        LogInfo(msg='  Topics:'),
        LogInfo(msg='    Input:  /kinect/depth/image_raw'),
        LogInfo(msg='    Output: /cmd_vel'),
        LogInfo(msg='    Debug:  /person_follower/debug_image'),
        LogInfo(msg='──────────────────────────────────────────────────────'),
        LogInfo(msg='  Para ajustar parámetros, edita:'),
        LogInfo(msg='    config/navigation.yaml'),
        LogInfo(msg='══════════════════════════════════════════════════════'),
    ])
