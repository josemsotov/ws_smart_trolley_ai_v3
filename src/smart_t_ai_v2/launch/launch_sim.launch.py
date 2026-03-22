import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Package name
    package_name = 'smart_t_ai_v2'

    # Include the robot_state_publisher launch file with file:// URIs
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'simulation': 'true'}.items()
    )

    pkg_share = get_package_share_directory(package_name)

    # Set Gazebo resource paths for worlds and meshes
    gazebo_resource_path_worlds = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'worlds')
    )

    gazebo_resource_path_pkg = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_share
    )

    # Include the Gazebo Sim launch file (ros_gz_sim)
    # Use server-only mode (-s) on RPi5 (no GPU for Gazebo GUI)
    # Use headless rendering for sensors
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': '-s -r obstacles.sdf',
        }.items()
    )

    # Spawn the robot in Gazebo Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'smart_trolley',
                   '-z', '0.1'],
        output='screen'
    )

    # Bridge for cmd_vel (ROS to Gazebo)
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen'
    )

    # Bridge for odometry
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )

    # Bridge for laser scan
    # DISABLED: Sensors system removed (RPi5 OpenGL 3.1 can't run ogre2)
    # No simulated LiDAR data — physics/odom/driving still work
    # bridge_scan = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
    #     output='screen'
    # )

    # Bridge for joint states
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/obstacles/model/smart_trolley/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen',
        remappings=[
            ('/world/obstacles/model/smart_trolley/joint_state', '/joint_states')
        ]
    )

    # TF publisher: converts /odom to odom->base_link TF
    # (DiffDrive plugin's native TF can't be reliably bridged)
    odom_tf_publisher = Node(
        package='smart_t_ai_v2',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Teleop twist keyboard for robot control
    # Ejecuta manualmente en tu terminal:
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_raw
    # teleop_keyboard = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_twist_keyboard',
    #     output='screen'
    # )

    # RViz node for simulation visualization
    rviz_config_file = os.path.join(pkg_share, 'config', 'view_bot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # ─── Joystick (evdev) ───
    joy_node = Node(
        package='smart_t_ai_v2',
        executable='joy_evdev_node.py',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ─── Stadia Teleop (D-pad + sticks) ───
    stadia_teleop = Node(
        package='smart_t_ai_v2',
        executable='stadia_teleop_node.py',
        name='stadia_teleop',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Launch them all!
    return LaunchDescription([
        gazebo_resource_path_worlds,
        gazebo_resource_path_pkg,
        rsp,
        gazebo,
        spawn_entity,
        bridge_cmd_vel,
        bridge_odom,
        # bridge_scan,  # Disabled — no sensors system on RPi5
        bridge_joint_states,
        odom_tf_publisher,
        rviz_node,
        joy_node,
        stadia_teleop,
    ])

