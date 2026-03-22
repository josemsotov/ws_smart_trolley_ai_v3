import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Wrapper ligero: RSP + Arduino + RViz (sin LiDAR, sin cámara, sin teleop).

    Equivale a:
      ros2 launch smart_t_ai_v2 launch_robot.launch.py use_lidar:=false use_camera:=false
    """
    pkg_smart_t = get_package_share_directory('smart_t_ai_v2')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_smart_t, 'launch', 'launch_robot.launch.py')
        ),
        launch_arguments={
            'use_lidar': 'false',
            'use_camera': 'false',
            'teleop': 'none',
        }.items()
    )

    return LaunchDescription([
        LogInfo(msg='══════════════════════════════════════════════'),
        LogInfo(msg='  SMART TROLLEY V5 - RViz Only (no peripherals)'),
        LogInfo(msg='══════════════════════════════════════════════'),
        robot_launch,
    ])
