// Copyright 2025 Smart Trolley V5 — Jose Soto Villasmil
// SPDX-License-Identifier: Apache-2.0
//
// TrolleyHardwareInterface — ros2_control SystemInterface for Arduino Mega 2560
//
// Protocol (MOTOR-INTERFACE-V-13):
//   write  →  "v <linear_m/s> <angular_rad/s>\n"
//   read   ←  "e <left_ticks> <right_ticks>\n"
//   IMU    ←  "i <yaw_dps> <accel_x_g>\n"      (Profile A only)
//   GPS    ←  "g <lat> <lon> <fix> <sats> <hdop> [alt] [speed]\n"
//   status ←  "s <firmware_string>\n"
//   reset  →  "r\n"

#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/string.hpp"

namespace smart_t_ai_v2
{

class TrolleyHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrolleyHardwareInterface)

  // ── Lifecycle ──────────────────────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── ros2_control interfaces ────────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Serial helpers ─────────────────────────────────────────────────────────
  bool openSerial();
  void closeSerial();
  void serialWrite(const char * buf, std::size_t n);  // ignores ssize_t return

  // ── Background read thread ─────────────────────────────────────────────────
  void serialReadLoop();
  void processLine(const std::string & line);

  // ── Sensor publishers ──────────────────────────────────────────────────────
  void publishImu();
  void publishGps();

  // ── Parameters (from URDF <hardware> block) ────────────────────────────────
  std::string serial_port_      = "/dev/ttyACM0";
  int         baud_rate_        = 115200;
  double      encoder_ppr_      = 85.0;  // 45 Profile-A · 85 Profile-B
  double      wheel_radius_     = 0.10;  // m
  double      wheel_separation_ = 0.48;  // m
  bool        imu_enabled_      = false; // Profile A (Hall+MPU9250)
  bool        gps_enabled_      = false;
  double      imu_rate_hz_      = 10.0;
  double      gps_rate_hz_      = 1.0;
  bool        motor_dir_invert_ = false;

  // ── Serial fd ──────────────────────────────────────────────────────────────
  int serial_fd_ = -1;

  // write-side lock (read thread never needs to write to serial)
  std::mutex write_mutex_;

  // ── Joint state (index 0 = left joint_lh, 1 = right joint_rh) ─────────────
  std::vector<double> hw_positions_  = {0.0, 0.0};  // rad  (accumulated)
  std::vector<double> hw_velocities_ = {0.0, 0.0};  // rad/s
  std::vector<double> hw_commands_   = {0.0, 0.0};  // rad/s  (from controller)

  // Latest encoder ticks from Arduino (protected by enc_mutex_)
  long enc_left_  = 0;
  long enc_right_ = 0;
  bool enc_init_  = false;
  std::mutex enc_mutex_;

  // ── IMU (Profile A) ────────────────────────────────────────────────────────
  double imu_yaw_dps_   = 0.0;  // deg/s
  double imu_accel_x_g_ = 0.0;  // g

  // ── GPS ────────────────────────────────────────────────────────────────────
  double gps_lat_   = 0.0;
  double gps_lon_   = 0.0;
  double gps_alt_   = 0.0;
  int    gps_fix_   = 0;
  int    gps_sats_  = 0;
  double gps_hdop_  = 99.9;
  double gps_speed_ = 0.0;

  // ── Read thread ────────────────────────────────────────────────────────────
  std::thread       serial_thread_;
  std::atomic<bool> running_{false};

  // ── Sensor node + executor (for IMU/GPS/status publishing) ────────────────
  rclcpp::Node::SharedPtr                                    sensor_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr       sensor_exec_;
  std::thread                                                sensor_exec_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr  gps_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        gps_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        status_pub_;

  // ── Sensor request timing ──────────────────────────────────────────────────
  using Clock = std::chrono::steady_clock;
  Clock::time_point last_imu_req_{Clock::now()};
  Clock::time_point last_gps_req_{Clock::now()};
};

}  // namespace smart_t_ai_v2
