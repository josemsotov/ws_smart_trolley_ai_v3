#!/usr/bin/env python3
"""
Launch file: Sistema de Interacción con Hailo AI HAT2+
=======================================================

Lanza los tres nodos de interacción del Smart Trolley:
  1. hailo_face_gesture_node  — Detección cara (Hailo SCRFD) + gestos (MediaPipe)
  2. voice_control_node       — Reconocimiento de voz offline (Vosk ES)
  3. robot_mode_manager_node  — Árbitro de modos + cmd_vel

Uso básico (todos los nodos):
  ros2 launch smart_t_ai_v2 interaction.launch.py

Solo cara + gestos (sin voz):
  ros2 launch smart_t_ai_v2 interaction.launch.py use_voice:=false

Solo voz (sin Hailo):
  ros2 launch smart_t_ai_v2 interaction.launch.py use_hailo:=false

Sin TTS:
  ros2 launch smart_t_ai_v2 interaction.launch.py tts_enabled:=false

Modo inicial diferente:
  ros2 launch smart_t_ai_v2 interaction.launch.py default_mode:=GESTURE

Tópicos principales:
  /robot/mode           — modo actual del robot (IDLE/TELEOP/GESTURE/VOICE/FOLLOW)
  /cmd_vel              — velocidad final al robot (desde mode_manager)
  /hailo/annotated      — imagen anotada con detecciones
  /voice/transcript     — transcripción de voz en tiempo real
  /robot/tts_say        — texto que dice el robot
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('smart_t_ai_v2')
    config = os.path.join(pkg, 'config', 'perception.yaml')

    # ─── Launch arguments ───────────────────────────────────────────────────
    use_hailo = LaunchConfiguration('use_hailo')
    use_voice = LaunchConfiguration('use_voice')
    use_manager = LaunchConfiguration('use_manager')
    default_mode = LaunchConfiguration('default_mode')
    tts_enabled = LaunchConfiguration('tts_enabled')
    require_auth = LaunchConfiguration('require_auth')
    publish_image = LaunchConfiguration('publish_image')
    process_rate = LaunchConfiguration('process_rate')
    device_index = LaunchConfiguration('device_index')

    declare_use_hailo = DeclareLaunchArgument(
        'use_hailo', default_value='true',
        description='Lanzar nodo de detección de cara y gestos (Hailo + MediaPipe)')

    declare_use_voice = DeclareLaunchArgument(
        'use_voice', default_value='true',
        description='Lanzar nodo de reconocimiento de voz (Vosk ES)')

    declare_use_manager = DeclareLaunchArgument(
        'use_manager', default_value='true',
        description='Lanzar nodo manager de modos y árbitro de cmd_vel')

    declare_default_mode = DeclareLaunchArgument(
        'default_mode', default_value='IDLE',
        description='Modo inicial del robot (IDLE/TELEOP/GESTURE/VOICE/FOLLOW)')

    declare_tts = DeclareLaunchArgument(
        'tts_enabled', default_value='true',
        description='Habilitar TTS (espeak) para anuncios de modo')

    declare_require_auth = DeclareLaunchArgument(
        'require_auth', default_value='true',
        description='Requerir cara autorizada para activar gestos')

    declare_publish_image = DeclareLaunchArgument(
        'publish_image', default_value='true',
        description='Publicar imagen anotada con detecciones en /hailo/annotated')

    declare_process_rate = DeclareLaunchArgument(
        'process_rate', default_value='10.0',
        description='Frecuencia de procesamiento del nodo Hailo (Hz)')

    declare_device_index = DeclareLaunchArgument(
        'device_index', default_value='-1',
        description='Índice de dispositivo de micrófono (-1 = automático)')

    # ─── Nodo 1: Hailo Face Detection + MediaPipe Gesture ───────────────────
    hailo_face_gesture = Node(
        package='smart_t_ai_v2',
        executable='hailo_face_gesture_node',
        name='hailo_face_gesture',
        output='screen',
        parameters=[
            config,
            {
                'require_auth':   require_auth,
                'publish_image':  publish_image,
                'process_rate':   process_rate,
            },
        ],
        condition=IfCondition(use_hailo),
    )

    # ─── Nodo 2: Voice Control (Vosk ES) ────────────────────────────────────
    voice_control = Node(
        package='smart_t_ai_v2',
        executable='voice_control_node',
        name='voice_control',
        output='screen',
        parameters=[
            config,
            {
                'device_index': device_index,
            },
        ],
        condition=IfCondition(use_voice),
    )

    # ─── Nodo 3: Robot Mode Manager ─────────────────────────────────────────
    mode_manager = Node(
        package='smart_t_ai_v2',
        executable='robot_mode_manager_node',
        name='robot_mode_manager',
        output='screen',
        parameters=[
            config,
            {
                'default_mode': default_mode,
                'tts_enabled':  tts_enabled,
            },
        ],
        remappings=[
            # El mode manager publica el /cmd_vel final
            ('/cmd_vel', '/cmd_vel'),
            # El joystick publica en /joy_cmd_vel (desde stadia_teleop_node)
            ('/joy_cmd_vel', '/joy_cmd_vel'),
        ],
        condition=IfCondition(use_manager),
    )

    # ─── LaunchDescription ──────────────────────────────────────────────────
    ld = LaunchDescription()

    # Argumentos
    ld.add_action(declare_use_hailo)
    ld.add_action(declare_use_voice)
    ld.add_action(declare_use_manager)
    ld.add_action(declare_default_mode)
    ld.add_action(declare_tts)
    ld.add_action(declare_require_auth)
    ld.add_action(declare_publish_image)
    ld.add_action(declare_process_rate)
    ld.add_action(declare_device_index)

    # Info
    ld.add_action(LogInfo(msg='══════════════════════════════════════════════════'))
    ld.add_action(LogInfo(msg='  🤖 Smart Trolley V5 — Sistema de Interacción'))
    ld.add_action(LogInfo(msg='  👁️  Hailo SCRFD + MediaPipe Hands'))
    ld.add_action(LogInfo(msg='  🎤 Vosk Español (offline)'))
    ld.add_action(LogInfo(msg='  🎛️  Mode Manager (IDLE/TELEOP/GESTURE/VOICE/FOLLOW)'))
    ld.add_action(LogInfo(msg='──────────────────────────────────────────────────'))
    ld.add_action(LogInfo(msg='  Tópicos clave:'))
    ld.add_action(LogInfo(msg='    /robot/mode       → modo actual'))
    ld.add_action(LogInfo(msg='    /cmd_vel          → velocidad final'))
    ld.add_action(LogInfo(msg='    /hailo/annotated  → imagen con detecciones'))
    ld.add_action(LogInfo(msg='    /voice/transcript → transcripción en vivo'))
    ld.add_action(LogInfo(msg='══════════════════════════════════════════════════'))

    # Nodos
    ld.add_action(hailo_face_gesture)
    ld.add_action(voice_control)
    ld.add_action(mode_manager)

    return ld
