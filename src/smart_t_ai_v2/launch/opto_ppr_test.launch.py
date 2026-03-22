#!/usr/bin/env python3
"""
opto_ppr_test.launch.py — Smart Trolley V5
Launch para la prueba de confirmación de PPR del encoder óptico (2 fases).

╔═══════════════════════════════════════════════════════════════════╗
║  PROCEDIMIENTO COMPLETO EN 2 FASES                               ║
╠═══════════════════════════════════════════════════════════════════╣
║  FASE 1 — Referencia Hall (perfil A, 45 PPR)                     ║
║    1. Coloca el robot en la posición de salida                    ║
║    2. Lanza:                                                      ║
║       ros2 launch smart_t_ai_v2 opto_ppr_test.launch.py \\       ║
║           profile:=A                                             ║
║    3. El robot avanzará y guardará la referencia odom en         ║
║       /tmp/opto_ppr_phase1.json                                  ║
║    4. Anota la instrucción que aparece al final de los logs      ║
║                                                                   ║
║  FASE 2 — Combinado Hall + Opto (perfil B, trial PPR)            ║
║    1. Lleva el robot de VUELTA al punto de inicio                ║
║    2. Detén el launch anterior (Ctrl+C)                          ║
║    3. Lanza:                                                      ║
║       ros2 launch smart_t_ai_v2 opto_ppr_test.launch.py \\       ║
║           profile:=B  trial_ppr:=117                             ║
║       (117 = Hall 45 + Opto 72 estimado)                         ║
║    4. El robot avanzará y calculará el PPR real del opto         ║
║                                                                   ║
║  RESULTADO:  /tmp/opto_ppr_results.json                          ║
╚═══════════════════════════════════════════════════════════════════╝
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('smart_t_ai_v2')

    # ── Paths ─────────────────────────────────────────────────────────
    arduino_yaml = os.path.join(pkg, 'config', 'arduino_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────
    profile      = LaunchConfiguration('profile')
    trial_ppr    = LaunchConfiguration('trial_ppr')
    serial_port  = LaunchConfiguration('serial_port')
    drive_speed  = LaunchConfiguration('drive_speed')
    drive_time   = LaunchConfiguration('drive_time')

    declare_profile = DeclareLaunchArgument(
        'profile', default_value='A',
        description="'A' = Fase 1 referencia Hall | 'B' = Fase 2 Hall+Opto")

    declare_trial_ppr = DeclareLaunchArgument(
        'trial_ppr', default_value='117',
        description='PPR total de prueba (Hall 45 + Opto 72 estimado = 117)')

    declare_serial = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Puerto serie del Arduino')

    declare_drive_speed = DeclareLaunchArgument(
        'drive_speed', default_value='0.10',
        description='Velocidad de avance para el test (m/s) — recomendado ≤ 0.12')

    declare_drive_time = DeclareLaunchArgument(
        'drive_time', default_value='6.0',
        description='Duración del avance en cada fase (segundos)')

    # ── Nodo arduino_bridge ───────────────────────────────────────────
    # En Fase A: encoder_ppr=45, sensor_profile=A (Hall reference)
    # En Fase B: encoder_ppr=trial_ppr, sensor_profile=B (Hall+Opto)
    #
    # Nota: 'profile' determina ambos parámetros automáticamente.
    # Para Phase A siempre usamos ppr=45 (hardcoded, referencia conocida).
    # Para Phase B usamos trial_ppr del argumento de launch.
    #
    # El selector se implementa pasando profile como sensor_profile y
    # dejando que el usuario use el argumento correcto.

    arduino_node = Node(
        package='smart_t_ai_v2',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[
            arduino_yaml,
            {
                'serial_port':      serial_port,
                'sensor_profile':   profile,          # 'A' ó 'B'
                'encoder_ppr':      trial_ppr,        # Fase A usa 45 (default yaml), Fase B usa trial
                'connection_type':  'usb',
                'publish_rate':     50.0,
                'encoder_read_rate': 10.0,
                'pid_heading_enabled': False,         # OFF para medir puro, sin corrección
            }
        ],
    )

    # ── Nodo de calibración ───────────────────────────────────────────
    # Se selecciona phase automáticamente según 'profile':
    #   profile=A → phase=1   profile=B → phase=2
    # No hay IfCondition para enteros simples, usamos el mismo nodo
    # con parámetros distintos según lo que el usuario especifique.
    # La selección de fase se hace via el parámetro 'phase' implícito:
    # perfil A → forzamos phase=1 con substitution.
    # Usamos dos bloques y ConditionOnEnvironmentVariable no disponible,
    # por eso el usuario debe pasar 'phase' manualmente o usar profile.
    # → El nodo opto_ppr_confirm deduce la fase desde el parámetro 'phase'.

    calib_node = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='smart_t_ai_v2',
                executable='opto_ppr_confirm',
                name='opto_ppr_confirm',
                output='screen',
                parameters=[{
                    'trial_ppr':    trial_ppr,
                    'hall_ppr':     45,
                    'drive_speed':  drive_speed,
                    'drive_time':   drive_time,
                    'wheel_radius': 0.10,
                    # NOTA: 'phase' se pasa desde CLI cuando se necesita
                    # Si profile=A → agrega  --ros-args -p phase:=1  (automático via launch)
                    # Si profile=B → agrega  --ros-args -p phase:=2
                    # El launch lo pasa en additional_arguments más abajo.
                }],
            )
        ]
    )

    # ── Nodo calibración Fase 1 (profile A) ──────────────────────────
    calib_phase1 = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='smart_t_ai_v2',
                executable='opto_ppr_confirm',
                name='opto_ppr_confirm',
                output='screen',
                parameters=[{
                    'phase':        1,
                    'trial_ppr':    trial_ppr,
                    'hall_ppr':     45,
                    'drive_speed':  drive_speed,
                    'drive_time':   drive_time,
                    'wheel_radius': 0.10,
                }],
            )
        ]
    )

    # ── Nodo calibración Fase 2 (profile B) ──────────────────────────
    calib_phase2 = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='smart_t_ai_v2',
                executable='opto_ppr_confirm',
                name='opto_ppr_confirm',
                output='screen',
                parameters=[{
                    'phase':        2,
                    'trial_ppr':    trial_ppr,
                    'hall_ppr':     45,
                    'drive_speed':  drive_speed,
                    'drive_time':   drive_time,
                    'wheel_radius': 0.10,
                }],
            )
        ]
    )

    # ── Selección de fase ─────────────────────────────────────────────
    # Como 'profile' es un string de LaunchConfiguration no podemos hacer
    # if/else en Python puro; usamos la aproximación de dos nodos separados
    # y seleccionamos con IfCondition sobre el argumento 'profile'.
    from launch.conditions import IfCondition, UnlessCondition
    from launch.substitutions import PythonExpression

    is_profile_a = IfCondition(
        PythonExpression(["'", profile, "' == 'A'"]))
    is_profile_b = IfCondition(
        PythonExpression(["'", profile, "' == 'B'"]))

    calib_phase1_cond = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='smart_t_ai_v2',
                executable='opto_ppr_confirm',
                name='opto_ppr_confirm',
                output='screen',
                condition=is_profile_a,
                parameters=[{
                    'phase':        1,
                    'trial_ppr':    trial_ppr,
                    'hall_ppr':     45,
                    'drive_speed':  drive_speed,
                    'drive_time':   drive_time,
                    'wheel_radius': 0.10,
                }],
            )
        ]
    )

    calib_phase2_cond = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='smart_t_ai_v2',
                executable='opto_ppr_confirm',
                name='opto_ppr_confirm',
                output='screen',
                condition=is_profile_b,
                parameters=[{
                    'phase':        2,
                    'trial_ppr':    trial_ppr,
                    'hall_ppr':     45,
                    'drive_speed':  drive_speed,
                    'drive_time':   drive_time,
                    'wheel_radius': 0.10,
                }],
            )
        ]
    )

    # ── arduino_bridge Fase A (encoder_ppr=45 fijo) ───────────────────
    arduino_phase_a = Node(
        package='smart_t_ai_v2',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        condition=is_profile_a,
        parameters=[
            arduino_yaml,
            {
                'serial_port':          serial_port,
                'sensor_profile':       'A',
                'encoder_ppr':          45,       # Hall reference, fijo
                'connection_type':      'usb',
                'publish_rate':         50.0,
                'encoder_read_rate':    10.0,
                'pid_heading_enabled':  False,    # sin PID para test puro
            }
        ],
    )

    # ── arduino_bridge Fase B (encoder_ppr=trial_ppr) ─────────────────
    arduino_phase_b = Node(
        package='smart_t_ai_v2',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        condition=is_profile_b,
        parameters=[
            arduino_yaml,
            {
                'serial_port':          serial_port,
                'sensor_profile':       'B',
                'encoder_ppr':          trial_ppr,   # ajustable
                'connection_type':      'usb',
                'publish_rate':         50.0,
                'encoder_read_rate':    10.0,
                'pid_heading_enabled':  False,
            }
        ],
    )

    # ── Log de inicio ──────────────────────────────────────────────────
    log_start = LogInfo(
        msg=['━━━ Opto PPR Test ━━━  ',
             'Profile: ', profile, '  |  ',
             'Trial PPR: ', trial_ppr, '  |  ',
             'Speed: ', drive_speed, ' m/s × ', drive_time, ' s']
    )

    return LaunchDescription([
        declare_profile,
        declare_trial_ppr,
        declare_serial,
        declare_drive_speed,
        declare_drive_time,
        log_start,
        arduino_phase_a,
        arduino_phase_b,
        calib_phase1_cond,
        calib_phase2_cond,
    ])
