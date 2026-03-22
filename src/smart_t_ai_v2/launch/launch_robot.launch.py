import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, TimerAction,
    ExecuteProcess, RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch COMPLETO para el robot real Smart Trolley V5.

    Lanza:
      - Robot State Publisher (URDF con meshes)
      - Arduino Bridge (motores, encoders, odometría, TF)
      - LiDAR LD06 (condicional)
      - Cámara USB v4l2 (condicional)
      - RViz2 (condicional)
      - Teleop: teclado o joystick (condicional)

    Uso básico:
      ros2 launch smart_t_ai_v2 launch_robot.launch.py

    Con todos los periféricos + control por teclado:
      ros2 launch smart_t_ai_v2 launch_robot.launch.py use_camera:=true teleop:=keyboard

    Con joystick DJI / Xbox:
      ros2 launch smart_t_ai_v2 launch_robot.launch.py teleop:=joystick
    """
    pkg_smart_t = get_package_share_directory('smart_t_ai_v2')

    # ─── Paths ───
    xacro_file      = os.path.join(pkg_smart_t, 'description', 'robot.urdf.xacro')
    config_file     = os.path.join(pkg_smart_t, 'config', 'hardware.yaml')
    controllers_cfg = os.path.join(pkg_smart_t, 'config', 'controllers.yaml')
    rviz_config     = os.path.join(pkg_smart_t, 'config', 'rviz', 'full_robot.rviz')
    stadia_config   = os.path.join(pkg_smart_t, 'config', 'control.yaml')

    # ─── Launch Arguments ───
    use_rviz = LaunchConfiguration('use_rviz')
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    use_kinect = LaunchConfiguration('use_kinect')
    use_coral = LaunchConfiguration('use_coral')
    use_gesture = LaunchConfiguration('use_gesture')
    use_interaction = LaunchConfiguration('use_interaction')
    teleop = LaunchConfiguration('teleop')
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')
    camera_device = LaunchConfiguration('camera_device')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz for visualization')

    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar', default_value='true',
        description='Launch LiDAR LD06 node')

    declare_use_camera = DeclareLaunchArgument(
        'use_camera', default_value='false',
        description='Launch USB camera node (disabled: Kinect RGB replaces it)')

    declare_use_kinect = DeclareLaunchArgument(
        'use_kinect', default_value='true',
        description='Launch Kinect Xbox 360 node (depth + RGB)')

    declare_use_coral = DeclareLaunchArgument(
        'use_coral', default_value='true',
        description='Launch Coral EdgeTPU detector node')

    declare_use_gesture = DeclareLaunchArgument(
        'use_gesture', default_value='false',
        description='Launch gesture control node (face recognition + hand gestures)')

    declare_use_interaction = DeclareLaunchArgument(
        'use_interaction', default_value='false',
        description='Launch Hailo interaction system (face+gesture+voice+mode_manager)')

    declare_teleop = DeclareLaunchArgument(
        'teleop', default_value='joystick',
        description='Teleop mode: none, keyboard, joystick (default: joystick/Stadia)')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Arduino serial port')

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='LiDAR LD06 serial port')

    declare_camera_device = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='Camera video device')

    # ─── Robot Description (real hardware — TrolleyHardwareInterface) ───
    # sim_mode:=false selects the C++ hardware interface plugin in ros2_control.xacro
    robot_description = Command([
        'xacro ', xacro_file,
        ' sim_mode:=false',
        ' serial_port:=', serial_port,
        ' use_camera:=', use_camera,
        ' use_lidar:=', use_lidar,
        ' use_kinect:=', use_kinect,
    ])

    # ─── Robot State Publisher ───
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'publish_frequency': 50.0,
        }]
    )

    # ─── ros2_control: Standalone Controller Manager (real hardware) ───
    # Loads TrolleyHardwareInterface plugin, which owns the serial port.
    # Remaps diff_drive_controller's cmd_vel_unstamped → /cmd_vel
    # so existing teleop nodes (Stadia, keyboard) work unchanged.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            controllers_cfg,
        ],
    )

    # ─── Controller Spawners (delayed to wait for controller_manager) ───
    joint_state_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster',
                           '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    diff_drive_spawner = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'diff_drive_controller',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            )
        ]
    )

    # ─── LiDAR LD06 (custom node) ───
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

    # ─── Camera USB (v4l2_camera) ───
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': camera_device,
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'camera_link_optical',
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        condition=IfCondition(use_camera),
    )

    # ─── Kinect Xbox 360 (freenect node) ───
    # Custom node using libfreenect via ctypes
    # Publishes: /kinect/rgb/image_raw, /kinect/depth/image_raw + camera_info
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

    # ─── Depth to LaserScan (Kinect depth → 2D scan) ───
    # Converts Kinect depth image to a virtual 2D laser scan
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
            ('depth', '/kinect/depth/image_raw'),
            ('depth_camera_info', '/kinect/depth/camera_info'),
            ('scan', '/kinect/scan'),
        ],
        condition=IfCondition(use_kinect),
    )

    # ─── Coral EdgeTPU Object Detector ───
    coral_detector = Node(
        package='smart_t_ai_v2',
        executable='coral_detector_node.py',
        name='coral_detector',
        output='screen',
        parameters=[{
            'image_topic': '/kinect/rgb/image_raw',
            'score_threshold': 0.4,
            'max_detections': 10,
            'publish_rate': 10.0,
            'publish_annotated': True,
            'use_edgetpu': True,
        }],
        condition=IfCondition(use_coral),
    )

    # ─── Gesture Control (face recognition + hand gestures → cmd_vel) ───
    gesture_control = Node(
        package='smart_t_ai_v2',
        executable='gesture_control_node.py',
        name='gesture_control',
        output='screen',
        parameters=[{
            'image_topic': '/kinect/rgb/image_raw',
            'process_rate': 8.0,
            'face_confidence': 55.0,
            'auth_timeout': 15.0,
            'gesture_hold_time': 0.6,
            'linear_speed': 0.2,
            'angular_speed': 0.5,
            'publish_image': True,
            'require_face': True,
            'toggle_btn': 2,             # Stadia X button
        }],
        remappings=[
            ('/gesture/cmd_vel', '/cmd_vel'),
        ],
        condition=IfCondition(use_gesture),
    )

    # ─── Hailo Interaction System (cara + gestos + voz + mode manager) ───
    # Combina:
    #   - hailo_face_gesture_node: SCRFD (Hailo) + MediaPipe Hands
    #   - voice_control_node:      Vosk ES offline
    #   - robot_mode_manager_node: árbitro de modos y cmd_vel
    # Activa con: use_interaction:=true
    interaction_config = os.path.join(pkg_smart_t, 'config', 'perception.yaml')

    hailo_face_gesture = Node(
        package='smart_t_ai_v2',
        executable='hailo_face_gesture_node.py',
        name='hailo_face_gesture',
        output='screen',
        parameters=[interaction_config],
        condition=IfCondition(use_interaction),
    )

    voice_control = Node(
        package='smart_t_ai_v2',
        executable='voice_control_node.py',
        name='voice_control',
        output='screen',
        parameters=[interaction_config],
        condition=IfCondition(use_interaction),
    )

    mode_manager = Node(
        package='smart_t_ai_v2',
        executable='robot_mode_manager_node.py',
        name='robot_mode_manager',
        output='screen',
        parameters=[interaction_config],
        # Redirige /cmd_vel del joystick (stadia → /joy_cmd_vel → mode_manager → /cmd_vel)
        remappings=[
            ('/joy_cmd_vel', '/joy_cmd_vel'),
        ],
        condition=IfCondition(use_interaction),
    )

    # ─── RViz2 ───
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(use_rviz),
    )

    # ─── Teleop: Keyboard (teleop:=keyboard) ───
    # teleop_twist_keyboard needs stdin, so we open it in a separate terminal window
    teleop_keyboard = TimerAction(
        period=3.0,  # Wait for Arduino Bridge to initialize
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--title=🎮 Teleop Keyboard - Smart Trolley',
                    '--', 'bash', '-c',
                    'echo "══════════════════════════════════════════════════"; '
                    'echo "  🎮 TELEOP KEYBOARD - Smart Trolley V5"; '
                    'echo "══════════════════════════════════════════════════"; '
                    'echo ""; '
                    'echo "  Controles:"; '
                    'echo "    u  i  o      ↖ ↑ ↗"; '
                    'echo "    j  k  l      ← ● →"; '
                    'echo "    m  ,  .      ↙ ↓ ↘"; '
                    'echo ""; '
                    'echo "  q/z : +/- velocidad linear  (10%)"; '
                    'echo "  w/x : +/- velocidad angular (10%)"; '
                    'echo "  k   : parar"; '
                    'echo "  Ctrl+C : salir"; '
                    'echo "══════════════════════════════════════════════════"; '
                    'echo ""; '
                    'source /opt/ros/jazzy/setup.bash && '
                    'source ' + os.path.join(os.path.dirname(pkg_smart_t), '..', '..', 'setup.bash') + ' 2>/dev/null; '
                    'ros2 run teleop_twist_keyboard teleop_twist_keyboard; '
                    'echo ""; echo "Teleop terminado. Presiona ENTER para cerrar..."; read'
                ],
                output='screen',
            )
        ],
        condition=IfCondition(PythonExpression(["'", teleop, "' == 'keyboard'"]))
    )

    # ─── Teleop: Joystick / Stadia Controller (teleop:=joystick) ───
    # Config: config/stadia_teleop.yaml
    # L1 = deadman | Stick Izq = mover | Stick Der = rotar Z
    # R1 = +velocidad | R2 = -velocidad
    joy_node = Node(
        package='smart_t_ai_v2',
        executable='joy_evdev_node.py',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_path': '',
            'autorepeat_rate': 20.0,
            'deadzone': 0.12,
        }],
        condition=IfCondition(PythonExpression(["'", teleop, "' == 'joystick'"]))
    )

    # Con use_interaction=true el stadia publica en /joy_cmd_vel para que
    # el mode_manager lo arbibre; sin él, publica directo en /cmd_vel.
    stadia_teleop_node = Node(
        package='smart_t_ai_v2',
        executable='stadia_teleop_node.py',
        name='stadia_teleop',
        output='screen',
        parameters=[stadia_config],
        remappings=[('/cmd_vel', '/joy_cmd_vel')],
        condition=IfCondition(PythonExpression(["'", teleop, "' == 'joystick'"]))
    )

    # ═══ Build LaunchDescription ═══
    ld = LaunchDescription()

    # Declare all arguments
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_lidar)
    ld.add_action(declare_use_camera)
    ld.add_action(declare_use_kinect)
    ld.add_action(declare_use_coral)
    ld.add_action(declare_use_gesture)
    ld.add_action(declare_use_interaction)
    ld.add_action(declare_teleop)
    ld.add_action(declare_serial_port)
    ld.add_action(declare_lidar_port)
    ld.add_action(declare_camera_device)

    # Info
    ld.add_action(LogInfo(msg='══════════════════════════════════════════════════'))
    ld.add_action(LogInfo(msg='  SMART TROLLEY V5 - Robot Real Launch'))
    ld.add_action(LogInfo(msg='  Teleop: joystick(Stadia)=default, keyboard, none'))
    ld.add_action(LogInfo(msg='  Stadia: L1=activar │ Izq=mover │ Der=rotar │ R1/R2=vel'))
    ld.add_action(LogInfo(msg='  use_interaction:=true → Hailo AI HAT2+ + voz + modos'))
    ld.add_action(LogInfo(msg='══════════════════════════════════════════════════'))

    # Core: RSP + ros2_control (TrolleyHardwareInterface owns serial port)
    ld.add_action(robot_state_publisher)
    ld.add_action(ros2_control_node)
    cmd_vel_relay = Node(
        package='smart_t_ai_v2',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
    )

    ld.add_action(joint_state_spawner)
    ld.add_action(diff_drive_spawner)
    ld.add_action(cmd_vel_relay)

    # Peripherals (conditional)
    ld.add_action(ldlidar_node)
    ld.add_action(camera_node)
    ld.add_action(kinect_node)
    ld.add_action(depth_to_scan)
    ld.add_action(coral_detector)
    ld.add_action(gesture_control)

    # Hailo Interaction System (condicional: use_interaction:=true)
    ld.add_action(hailo_face_gesture)
    ld.add_action(voice_control)
    ld.add_action(mode_manager)

    ld.add_action(rviz)

    # Teleop (conditional: keyboard or joystick)
    ld.add_action(teleop_keyboard)
    ld.add_action(joy_node)
    ld.add_action(stadia_teleop_node)

    return ld
