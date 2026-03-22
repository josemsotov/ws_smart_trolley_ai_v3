import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Wrapper: lanza launch_robot.launch.py con teleop activado.

    Por defecto usa el control Google Stadia (joystick).
    Si el Stadia no está conectado, usar:
      ros2 launch smart_t_ai_v2 teleop_robot.launch.py teleop:=keyboard

    Modos:
      teleop:=joystick   → Google Stadia (L1+stick=drive, R1=turbo)
      teleop:=keyboard   → Teclado (u/i/o j/k/l m/,/.)
      teleop:=none       → Sin teleop
    """
    pkg_smart_t = get_package_share_directory('smart_t_ai_v2')

    teleop_mode = LaunchConfiguration('teleop')
    declare_teleop = DeclareLaunchArgument(
        'teleop', default_value='joystick',
        description='Teleop mode: joystick (Stadia), keyboard, none')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_smart_t, 'launch', 'launch_robot.launch.py')
        ),
        launch_arguments={
            'teleop': teleop_mode,
            'use_camera': 'true',
        }.items()
    )

    return LaunchDescription([
        declare_teleop,
        LogInfo(msg='══════════════════════════════════════════════'),
        LogInfo(msg='  SMART TROLLEY V5 - Teleop Mode'),
        LogInfo(msg='  Default: Google Stadia (L1+stick, R1=turbo)'),
        LogInfo(msg='  Fallback: teleop:=keyboard'),
        LogInfo(msg='══════════════════════════════════════════════'),
        robot_launch,
    ])
