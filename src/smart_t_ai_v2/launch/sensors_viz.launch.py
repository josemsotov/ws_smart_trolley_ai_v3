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
    Launch para visualizar sensores (Kinect + LiDAR) en RViz sin odometría.
    
    Solo lanza:
      - Robot State Publisher (modelo URDF estático)
      - Kinect Node (RGB + Depth)
      - LiDAR LD06 
      - Depth to LaserScan (convierte depth a scan 2D)
      - RViz2 (visualización)
    
    Uso:
      ros2 launch smart_t_ai_v2 sensors_viz.launch.py
    
    Opciones:
      ros2 launch smart_t_ai_v2 sensors_viz.launch.py use_lidar:=false
      ros2 launch smart_t_ai_v2 sensors_viz.launch.py use_kinect:=false
    """
    pkg_smart_t = get_package_share_directory('smart_t_ai_v2')

    # ─── Paths ───
    xacro_file = os.path.join(pkg_smart_t, 'description', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_smart_t, 'config', 'sensors_view.rviz')

    # ─── Launch Arguments ───
    use_lidar = LaunchConfiguration('use_lidar')
    use_kinect = LaunchConfiguration('use_kinect')
    lidar_port = LaunchConfiguration('lidar_port')

    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar', default_value='true',
        description='Launch LiDAR LD06 node')

    declare_use_kinect = DeclareLaunchArgument(
        'use_kinect', default_value='true',
        description='Launch Kinect Xbox 360 node (depth + RGB)')

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='LiDAR LD06 serial port')

    # ─── Robot Description (sin odometría, solo visualización) ───
    robot_description = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=false',
        ' sim_mode:=false',
        ' use_camera:=false',
        ' use_lidar:=', use_lidar,
        ' use_kinect:=', use_kinect,
    ])

    # ─── Robot State Publisher (publica TF estáticos del robot) ───
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'publish_frequency': 10.0,  # Baja frecuencia, solo para TF estáticos
        }]
    )

    # ─── Static TF: base_link como raíz (sin movimiento, sin odometría) ───
    static_tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )

    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # ─── LiDAR LD06 ───
    ldlidar_node = Node(
        package='smart_t_ai_v2',
        executable='ld06_lidar_node.py',
        name='ld06_lidar',
        output='screen',
        parameters=[
            {'port': lidar_port},
            {'baudrate': 230400},
            {'topic_name': 'scan'},
            {'frame_id': 'laser_frame'},
            {'range_min': 0.02},
            {'range_max': 12.0},
            {'scan_direction': True},
            {'enable_angle_crop': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
        ],
        condition=IfCondition(use_lidar),
    )

    # ─── Kinect Xbox 360 (freenect node) ───
    kinect_node = Node(
        package='smart_t_ai_v2',
        executable='kinect_node.py',
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
        condition=IfCondition(use_kinect),
    )

    # ─── Depth to LaserScan (convierte Kinect depth a scan 2D) ───
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height': 10,
            'output_frame_id': 'kinect_depth_optical',
            'range_min': 0.45,
            'range_max': 10.0,
        }],
        remappings=[
            ('/depth', '/kinect/depth/image_raw'),
            ('/depth_camera_info', '/kinect/depth/camera_info'),
            ('/scan', '/kinect/scan'),
        ],
        condition=IfCondition(use_kinect),
    )

    # ─── RViz2 ───
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    )

    return LaunchDescription([
        LogInfo(msg='══════════════════════════════════════════════════════'),
        LogInfo(msg='  SENSORS VISUALIZATION - Kinect + LiDAR (No Odometry)'),
        LogInfo(msg='══════════════════════════════════════════════════════'),
        
        # Arguments
        declare_use_lidar,
        declare_use_kinect,
        declare_lidar_port,
        
        # Nodes
        robot_state_publisher,
        static_tf_map_to_base,
        static_tf_odom_to_base,
        ldlidar_node,
        kinect_node,
        depth_to_scan,
        rviz_node,
        
        LogInfo(msg='──────────────────────────────────────────────────────'),
        LogInfo(msg='  Topics:'),
        LogInfo(msg='    LiDAR:  /scan'),
        LogInfo(msg='    Kinect: /kinect/rgb/image_raw'),
        LogInfo(msg='            /kinect/depth/image_raw'),
        LogInfo(msg='            /kinect/scan (depth as 2D scan)'),
        LogInfo(msg='══════════════════════════════════════════════════════'),
    ])
