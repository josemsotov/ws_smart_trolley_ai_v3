cmake_minimum_required(VERSION 3.8)
project(smart_t_ai_v2)

# ── C++ standard ─────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ament packages ────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)

# ros2_control hardware interface + plugin system
find_package(hardware_interface REQUIRED)
find_package(pluginlib         REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(rclcpp_lifecycle  REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(std_msgs          REQUIRED)

# ── TrolleyHardwareInterface shared library ───────────────────────────────────
add_library(trolley_hardware_interface SHARED
  hardware/trolley_hardware_interface.cpp
)

target_include_directories(trolley_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(trolley_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

# Register pluginlib plugin description
pluginlib_export_plugin_description_file(
  hardware_interface trolley_hardware_interface.xml
)

# ── Install targets ────────────────────────────────────────────────────────────
install(
  TARGETS trolley_hardware_interface
  EXPORT  export_trolley_hardware_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY hardware/include/
  DESTINATION include
)

install(
  FILES trolley_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch worlds meshes scripts models
  DESTINATION share/${PROJECT_NAME}
)

# Instalar scripts Python como ejecutables sin extensión .py
# para que funcione: ros2 run smart_t_ai_v2 <nombre_sin_py>
set(PYTHON_SCRIPTS
  scripts/arduino_bridge.py
  scripts/bluetooth_helper.py
  scripts/coral_detector_node.py
  scripts/hailo_face_gesture_node.py
  scripts/voice_control_node.py
  scripts/robot_mode_manager_node.py
  scripts/joy_evdev_node.py
  scripts/kinect_node.py
  scripts/kinect_track.py
  scripts/ld06_lidar_node.py
  scripts/odom_tf_publisher.py
  scripts/person_follower.py
  scripts/fake_depth_publisher.py
  scripts/cmd_vel_dashboard.py
  scripts/register_face.py
  scripts/stadia_teleop_node.py
  scripts/cmd_vel_relay.py
  scripts/test_arduino_connection.py
  scripts/test_motors_hall.py
  scripts/test_positioning.py
  scripts/motor_simple_test.py
  scripts/motor_diag.py
  scripts/opto_ppr_confirm.py
  scripts/test_5m_rotate.py
)

foreach(script ${PYTHON_SCRIPTS})
  get_filename_component(script_name ${script} NAME_WE)
  install(PROGRAMS ${script}
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script_name}
  )
endforeach()

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(trolley_hardware_interface)
ament_export_targets(export_trolley_hardware_interface)

ament_package()
