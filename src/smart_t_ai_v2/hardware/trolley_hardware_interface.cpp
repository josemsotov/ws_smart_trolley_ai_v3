// Copyright 2025 Smart Trolley V5 — Jose Soto Villasmil
// SPDX-License-Identifier: Apache-2.0

#include "smart_t_ai_v2/trolley_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"

namespace smart_t_ai_v2
{

// ─────────────────────────────────────────────────────────────────────────────
// on_init — read parameters from URDF <hardware> block
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
TrolleyHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & p = info_.hardware_parameters;

  auto get_str = [&](const std::string & key, const std::string & def) -> std::string {
    return p.count(key) ? p.at(key) : def;
  };
  auto get_int = [&](const std::string & key, int def) -> int {
    return p.count(key) ? std::stoi(p.at(key)) : def;
  };
  auto get_dbl = [&](const std::string & key, double def) -> double {
    return p.count(key) ? std::stod(p.at(key)) : def;
  };
  auto get_bool = [&](const std::string & key, bool def) -> bool {
    if (!p.count(key)) return def;
    const auto & v = p.at(key);
    return (v == "true" || v == "1" || v == "True");
  };

  serial_port_      = get_str ("serial_port",          "/dev/ttyACM0");
  baud_rate_        = get_int ("baud_rate",             115200);
  encoder_ppr_      = get_dbl ("encoder_ppr",           85.0);
  wheel_radius_     = get_dbl ("wheel_radius",          0.10);
  wheel_separation_ = get_dbl ("wheel_separation",      0.48);
  imu_enabled_      = get_bool("imu_enabled",           false);
  gps_enabled_      = get_bool("gps_enabled",           false);
  imu_rate_hz_      = get_dbl ("imu_rate_hz",           10.0);
  gps_rate_hz_      = get_dbl ("gps_rate_hz",           1.0);
  motor_dir_invert_ = get_bool("motor_direction_invert", false);

  if (info_.joints.size() != 2)
  {
    RCLCPP_FATAL(get_logger(),
      "[TrolleyHW] Expected 2 joints (joint_lh, joint_rh), got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_  = {0.0, 0.0};
  hw_velocities_ = {0.0, 0.0};
  hw_commands_   = {0.0, 0.0};

  RCLCPP_INFO(get_logger(),
    "[TrolleyHW] on_init: port=%s baud=%d ppr=%.0f r=%.3fm sep=%.3fm IMU=%s GPS=%s",
    serial_port_.c_str(), baud_rate_, encoder_ppr_,
    wheel_radius_, wheel_separation_,
    imu_enabled_ ? "on" : "off", gps_enabled_ ? "on" : "off");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
TrolleyHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  // joint_lh → index 0
  si.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_positions_[0]);
  si.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]);
  // joint_rh → index 1
  si.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &hw_positions_[1]);
  si.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]);
  return si;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_command_interfaces
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
TrolleyHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
  ci.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);
  return ci;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure — open serial, create sensor publishers
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
TrolleyHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(get_logger(), "[TrolleyHW] Configuring...");

  if (!openSerial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Dedicated node for sensor topics (IMU, GPS, status)
  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  sensor_node_ = rclcpp::Node::make_shared("trolley_sensors", opts);

  if (imu_enabled_) {
    imu_pub_ = sensor_node_->create_publisher<sensor_msgs::msg::Imu>("/imu/raw", 10);
  }
  if (gps_enabled_) {
    gps_pub_        = sensor_node_->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    gps_status_pub_ = sensor_node_->create_publisher<std_msgs::msg::String>("/gps/status", 10);
  }
  status_pub_ = sensor_node_->create_publisher<std_msgs::msg::String>("/arduino_status", 10);

  sensor_exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  sensor_exec_->add_node(sensor_node_);
  sensor_exec_thread_ = std::thread([this] { sensor_exec_->spin(); });

  RCLCPP_INFO(get_logger(), "[TrolleyHW] Configured. Serial open: %s", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate — start read thread, reset encoders
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
TrolleyHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(get_logger(), "[TrolleyHW] Activating...");

  // Reset encoder counts on Arduino
  {
    std::lock_guard<std::mutex> lk(write_mutex_);
    serialWrite("r\n", 2);
  }
  usleep(200'000);  // 200 ms — wait for Arduino reset response

  {
    std::lock_guard<std::mutex> lk(enc_mutex_);
    enc_left_ = enc_right_ = 0;
    enc_init_ = false;
  }
  hw_positions_  = {0.0, 0.0};
  hw_velocities_ = {0.0, 0.0};
  hw_commands_   = {0.0, 0.0};

  running_ = true;
  serial_thread_ = std::thread(&TrolleyHardwareInterface::serialReadLoop, this);

  last_imu_req_ = Clock::now();
  last_gps_req_ = Clock::now();

  RCLCPP_INFO(get_logger(), "[TrolleyHW] Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_deactivate — stop motors, stop read thread
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
TrolleyHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(get_logger(), "[TrolleyHW] Deactivating...");

  {
    std::lock_guard<std::mutex> lk(write_mutex_);
    serialWrite("v 0.0 0.0\n", 10);
  }

  running_ = false;
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }

  RCLCPP_INFO(get_logger(), "[TrolleyHW] Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_cleanup — close serial, stop sensor executor
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
TrolleyHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*prev*/)
{
  if (sensor_exec_) {
    sensor_exec_->cancel();
  }
  if (sensor_exec_thread_.joinable()) {
    sensor_exec_thread_.join();
  }
  sensor_exec_.reset();
  sensor_node_.reset();

  closeSerial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// read — request encoder data, update joint states, schedule IMU/GPS requests
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type
TrolleyHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const auto now = Clock::now();

  // Request encoder tick snapshot
  {
    std::lock_guard<std::mutex> lk(write_mutex_);
    serialWrite("e\n", 2);
  }

  // Give the background reader ~3 ms to receive the response
  usleep(3'000);

  // Copy latest encoder values (thread-safe)
  long left_ticks, right_ticks;
  {
    std::lock_guard<std::mutex> lk(enc_mutex_);
    left_ticks  = enc_left_;
    right_ticks = enc_right_;
  }

  // Convert ticks → accumulated radians
  const double ticks_per_rad = encoder_ppr_ / (2.0 * M_PI);
  const double left_rad  = static_cast<double>(left_ticks)  / ticks_per_rad;
  const double right_rad = static_cast<double>(right_ticks) / ticks_per_rad;

  // Compute velocities from position delta / dt
  const double dt = period.seconds();
  if (enc_init_ && dt > 1e-6)
  {
    hw_velocities_[0] = (left_rad  - hw_positions_[0]) / dt;
    hw_velocities_[1] = (right_rad - hw_positions_[1]) / dt;
  }

  hw_positions_[0] = left_rad;
  hw_positions_[1] = right_rad;
  enc_init_ = true;

  // Schedule IMU request (Profile A)
  if (imu_enabled_)
  {
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_imu_req_).count();
    if (elapsed_ms >= static_cast<long>(1000.0 / imu_rate_hz_))
    {
      std::lock_guard<std::mutex> lk(write_mutex_);
      serialWrite("i\n", 2);
      last_imu_req_ = now;
    }
  }

  // Schedule GPS request
  if (gps_enabled_)
  {
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_gps_req_).count();
    if (elapsed_ms >= static_cast<long>(1000.0 / gps_rate_hz_))
    {
      std::lock_guard<std::mutex> lk(write_mutex_);
      serialWrite("g\n", 2);
      last_gps_req_ = now;
    }
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write — convert wheel velocities (rad/s) → linear/angular (m/s, rad/s),
//          send "v <lin> <ang>\n" to Arduino
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type
TrolleyHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double v_l = hw_commands_[0];  // rad/s  left
  double v_r = hw_commands_[1];  // rad/s  right

  if (motor_dir_invert_) {
    v_l = -v_l;
    v_r = -v_r;
  }

  // Differential kinematics: individual wheel vel → linear/angular
  const double linear  = wheel_radius_ * (v_l + v_r) / 2.0;        // m/s
  const double angular = wheel_radius_ * (v_r - v_l) / wheel_separation_;  // rad/s

  char cmd[64];
  std::snprintf(cmd, sizeof(cmd), "v %.4f %.4f\n", linear, angular);

  std::lock_guard<std::mutex> lk(write_mutex_);
  serialWrite(cmd, std::strlen(cmd));

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// serialWrite — helper that discards ssize_t to silence -Wunused-result
// ─────────────────────────────────────────────────────────────────────────────
void TrolleyHardwareInterface::serialWrite(const char * buf, std::size_t n)
{
  if (serial_fd_ < 0) { return; }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
  ::write(serial_fd_, buf, n);
#pragma GCC diagnostic pop
}

// ─────────────────────────────────────────────────────────────────────────────
// serialReadLoop — background thread: byte-by-byte read, dispatch complete lines
// ─────────────────────────────────────────────────────────────────────────────
void TrolleyHardwareInterface::serialReadLoop()
{
  std::string buf;
  buf.reserve(128);

  while (running_)
  {
    char c = 0;
    const int n = ::read(serial_fd_, &c, 1);
    if (n > 0)
    {
      if (c == '\n')
      {
        if (!buf.empty())
        {
          processLine(buf);
          buf.clear();
        }
      }
      else if (c != '\r')
      {
        buf += c;
      }
    }
    else
    {
      // No data → yield CPU briefly
      usleep(500);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// processLine — parse one line from the Arduino
// ─────────────────────────────────────────────────────────────────────────────
void TrolleyHardwareInterface::processLine(const std::string & line)
{
  if (line.size() < 2) { return; }

  const char type = line[0];

  // ── Encoders: "e <left_ticks> <right_ticks>" ───────────────────────────
  if (type == 'e' && line[1] == ' ')
  {
    long l = 0, r = 0;
    if (std::sscanf(line.c_str(), "e %ld %ld", &l, &r) == 2)
    {
      std::lock_guard<std::mutex> lk(enc_mutex_);
      enc_left_  = l;
      enc_right_ = r;
    }
    return;
  }

  // ── IMU: "i <yaw_dps> <accel_x_g>" ───────────────────────────────────
  if (type == 'i' && line[1] == ' ' && imu_enabled_)
  {
    double yaw = 0.0, ax = 0.0;
    if (std::sscanf(line.c_str(), "i %lf %lf", &yaw, &ax) == 2)
    {
      imu_yaw_dps_   = yaw;
      imu_accel_x_g_ = ax;
      publishImu();
    }
    return;
  }

  // ── GPS: "g <lat> <lon> <fix> <sats> <hdop> [alt] [speed]" ──────────
  if (type == 'g' && line[1] == ' ' && gps_enabled_)
  {
    double lat = 0, lon = 0, hdop = 99.9, alt = 0, spd = 0;
    int fix = 0, sats = 0;
    const int n = std::sscanf(line.c_str(), "g %lf %lf %d %d %lf %lf %lf",
                              &lat, &lon, &fix, &sats, &hdop, &alt, &spd);
    if (n >= 4)
    {
      gps_lat_   = lat;   gps_lon_   = lon;
      gps_fix_   = fix;   gps_sats_  = sats;
      gps_hdop_  = (n >= 5) ? hdop : 99.9;
      gps_alt_   = (n >= 6) ? alt  : 0.0;
      gps_speed_ = (n >= 7) ? spd  : 0.0;
      publishGps();
    }
    return;
  }

  // ── Status: "s <text>" ────────────────────────────────────────────────
  if (type == 's' && line[1] == ' ' && status_pub_)
  {
    std_msgs::msg::String msg;
    msg.data = line.substr(2);
    status_pub_->publish(msg);
    return;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// publishImu
// ─────────────────────────────────────────────────────────────────────────────
void TrolleyHardwareInterface::publishImu()
{
  if (!imu_pub_) { return; }

  sensor_msgs::msg::Imu msg;
  msg.header.stamp     = sensor_node_->now();
  msg.header.frame_id  = "imu_link";

  // Yaw rate deg/s → rad/s
  msg.angular_velocity.z = imu_yaw_dps_ * (M_PI / 180.0);
  msg.angular_velocity_covariance[8] = 0.02;

  // Accel g → m/s²
  msg.linear_acceleration.x = imu_accel_x_g_ * 9.81;
  msg.linear_acceleration_covariance[0] = 0.04;

  // Orientation unknown
  msg.orientation_covariance[0] = -1.0;

  imu_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// publishGps
// ─────────────────────────────────────────────────────────────────────────────
void TrolleyHardwareInterface::publishGps()
{
  if (!gps_pub_) { return; }

  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp    = sensor_node_->now();
  fix.header.frame_id = "gps_link";
  fix.latitude        = gps_lat_;
  fix.longitude       = gps_lon_;
  fix.altitude        = gps_alt_;

  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  fix.status.status  = (gps_fix_ >= 1)
    ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
    : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

  // Approximate covariance from HDOP (5 m/unit)
  const double hdop_m2 = std::pow(gps_hdop_ * 5.0, 2.0);
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  fix.position_covariance = {
    hdop_m2, 0.0,     0.0,
    0.0,     hdop_m2, 0.0,
    0.0,     0.0,     hdop_m2 * 4.0
  };

  gps_pub_->publish(fix);

  if (gps_status_pub_)
  {
    std_msgs::msg::String st;
    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "%s sats=%d hdop=%.1f lat=%.6f lon=%.6f alt=%.1f",
      (gps_fix_ >= 1 ? "FIX" : "NO_FIX"),
      gps_sats_, gps_hdop_, gps_lat_, gps_lon_, gps_alt_);
    st.data = buf;
    gps_status_pub_->publish(st);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// openSerial
// ─────────────────────────────────────────────────────────────────────────────
bool TrolleyHardwareInterface::openSerial()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(get_logger(), "[TrolleyHW] Cannot open %s: %s",
                 serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  struct termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(get_logger(), "[TrolleyHW] tcgetattr failed: %s", std::strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Select baud rate
  speed_t baud = B115200;
  switch (baud_rate_) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    default:
      RCLCPP_WARN(get_logger(), "[TrolleyHW] Unknown baud rate %d, defaulting to 115200", baud_rate_);
      baud = B115200;
  }
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  tty.c_cflag |= (CLOCAL | CREAD);  // Enable receiver, ignore modem status
  tty.c_cflag &= ~PARENB;           // No parity
  tty.c_cflag &= ~CSTOPB;           // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;               // 8 data bits

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // No software flow control
  tty.c_iflag &= ~(ICRNL | INLCR);         // No CR/NL translation

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw mode
  tty.c_oflag &= ~OPOST;                            // Raw output

  tty.c_cc[VMIN]  = 0;   // Non-blocking read
  tty.c_cc[VTIME] = 1;   // 0.1 s read timeout

  tcsetattr(serial_fd_, TCSANOW, &tty);
  tcflush(serial_fd_, TCIFLUSH);

  // Arduino auto-resets on USB connect; wait for it to boot
  usleep(2'000'000);  // 2 s

  RCLCPP_INFO(get_logger(), "[TrolleyHW] Serial port %s opened at %d baud",
              serial_port_.c_str(), baud_rate_);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// closeSerial
// ─────────────────────────────────────────────────────────────────────────────
void TrolleyHardwareInterface::closeSerial()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(get_logger(), "[TrolleyHW] Serial port closed.");
  }
}

}  // namespace smart_t_ai_v2

PLUGINLIB_EXPORT_CLASS(
  smart_t_ai_v2::TrolleyHardwareInterface,
  hardware_interface::SystemInterface)
