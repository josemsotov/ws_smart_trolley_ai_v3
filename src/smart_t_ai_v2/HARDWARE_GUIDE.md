# SMART TROLLEY - Hardware Integration Guide

## Overview

This package provides ROS2 integration for the SMART TROLLEY V5 robot with Arduino-based motor control. It includes both simulation (Gazebo) and real hardware control capabilities.

## Architecture

```
ROS2 Navigation Stack
        ↓
    /cmd_vel (Twist)
        ↓
  arduino_bridge.py ←→ Arduino Mega 2560
        ↓                    ↓
    /odom (Odometry)    Motor Control
    /joint_states       (ZS-X11H Drivers)
    /tf (odom→base)     Hall Sensors
```

## Hardware Requirements

- **Arduino Mega 2560** running MOTOR-INTERFACE-V-13 firmware
- **ZS-X11H Motor Drivers** (x2) for differential drive
- **Hall Effect Sensors** for encoder feedback
- **USB cable** for Arduino connection
- **Linux system** with ROS2 Jazzy

## Quick Start

### 1. Install Dependencies

```bash
# ROS2 packages (should already be installed)
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher

# Python serial library
pip3 install pyserial
```

### 2. Add User to Dialout Group

```bash
sudo usermod -a -G dialout $USER
# Logout and login for changes to take effect
```

### 3. Test Arduino Connection

```bash
# Check available serial ports
ls /dev/ttyACM* /dev/ttyUSB*

# Test connection (from workspace root)
cd ~/WS/ws_smart_trolley_ai_v2
python3 src/smart_t_ai_v2/scripts/test_arduino_connection.py /dev/ttyACM0
```

### 4. Build and Launch

```bash
# Build the package
cd ~/WS/ws_smart_trolley_ai_v2
colcon build --packages-select smart_t_ai_v2

# Source the workspace
source install/setup.bash

# Launch the hardware robot
ros2 launch smart_t_ai_v2 launch_robot.launch.py

# Or specify a different serial port
ros2 launch smart_t_ai_v2 launch_robot.launch.py serial_port:=/dev/ttyUSB0

# Launch without RViz
ros2 launch smart_t_ai_v2 launch_robot.launch.py use_rviz:=false
```

### 5. Control the Robot

```bash
# Install teleop (if not already installed)
sudo apt install ros-jazzy-teleop-twist-keyboard

# Control with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Configuration

Edit [config/arduino_params.yaml](config/arduino_params.yaml) to adjust:

- **Serial Port**: Default is `/dev/ttyACM0`
- **Wheel Parameters**: `wheel_separation` (0.48m), `wheel_radius` (0.10m)
- **Encoder Resolution**: `encoder_ppr` (pulses per revolution)
- **Velocity Limits**: `max_linear_vel`, `max_angular_vel`
- **Control Rates**: `publish_rate`, `encoder_read_rate`

## Arduino Protocol

The Arduino firmware uses a simple text-based protocol:

### Commands (PC → Arduino)

- `v <linear> <angular>` - Set velocity (m/s, rad/s)
- `e` - Request encoder readings
- `r` - Reset encoders to zero
- `s` - Request status
- `c` - Calibrate (if implemented)

### Responses (Arduino → PC)

- `e <left> <right>` - Encoder counts
- `s <status>` - Status message

### Example

```
PC → Arduino: v 0.5 0.0\n    (move forward at 0.5 m/s)
PC → Arduino: e\n            (request encoders)
Arduino → PC: e 1234 1240\n  (encoder counts)
```

## Comparison: Simulation vs Hardware

| Feature | Simulation | Hardware |
|---------|-----------|----------|
| **Launch** | `launch_sim.launch.py` | `launch_robot.launch.py` |
| **Physics** | Gazebo Sim | Real world |
| **Odometry** | Gazebo plugin | Arduino encoders |
| **Sensors** | Simulated | Real sensors |
| **Control** | `/cmd_vel` | `/cmd_vel` (same interface) |

## Monitoring

### Topics

```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor odometry
ros2 topic echo /odom

# Monitor Arduino status
ros2 topic echo /arduino_status

# Monitor encoder-based joint states
ros2 topic echo /joint_states
```

### TF Tree

```bash
# Visualize TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

Expected tree:
```
odom
 └─ base_link
     ├─ base_laser
     ├─ camera_link
     ├─ wheel_left_link
     └─ wheel_right_link
```

## Troubleshooting

### Arduino Not Detected

```bash
# Check USB connection
lsusb | grep Arduino

# Check serial ports
ls -l /dev/ttyACM* /dev/ttyUSB*

# Check permissions
groups | grep dialout  # Should show dialout

# If not in dialout group:
sudo usermod -a -G dialout $USER
# Then logout and login
```

### Robot Not Moving

1. **Check Arduino status topic**: `ros2 topic echo /arduino_status`
2. **Verify cmd_vel is being published**: `ros2 topic echo /cmd_vel`
3. **Test Arduino directly**: Use test script above
4. **Check motor connections**: Verify ZS-X11H wiring
5. **Check power supply**: Motors need adequate power

### Odometry Issues

1. **Check encoder readings**: `ros2 topic echo /joint_states`
2. **Verify encoder_ppr**: Must match actual Hall sensor resolution
3. **Calibrate wheel parameters**: Measure actual `wheel_separation` and `wheel_radius`
4. **Reset encoders**: `ros2 topic pub /reset_encoders std_msgs/msg/Empty "{}"` (if implemented)

### Serial Communication Errors

```bash
# Check baudrate matches Arduino firmware (115200)
# Check serial buffer:
cat /dev/ttyACM0  # Should see data (Ctrl+C to stop)

# Reset Arduino:
# Press reset button or disconnect/reconnect USB
```

## Development

### File Structure

```
smart_t_ai_v2/
├── config/
│   ├── arduino_params.yaml       # Hardware configuration
│   └── view_robot.rviz           # RViz config
├── description/
│   ├── SMART-TROLLEY_V5.xacro    # Robot URDF
│   ├── SMART-TROLLEY_V5.gazebo   # Gazebo config
│   └── gazebo_control_trolley.xacro
├── launch/
│   ├── launch_sim.launch.py      # Simulation
│   └── launch_robot.launch.py    # Hardware
├── scripts/
│   ├── arduino_bridge.py         # Hardware interface
│   ├── cmd_vel_inverter.py       # Simulation helper
│   └── test_arduino_connection.py
└── worlds/
    └── smart_t.sdf               # Gazebo world
```

### Modifying the Arduino Bridge

The [arduino_bridge.py](scripts/arduino_bridge.py) node handles:

- Serial communication with Arduino
- Velocity command translation
- Encoder reading and odometry calculation
- TF broadcasting (odom → base_link)
- Watchdog timer for safety

Key methods:
- `cmd_vel_callback()` - Handles `/cmd_vel` commands
- `read_encoders()` - Requests encoder data from Arduino
- `update_odometry()` - Calculates robot pose from encoders
- `publish_odom()` - Publishes odometry and TF

### Adding New Sensors

1. Wire sensor to Arduino
2. Update Arduino firmware to read sensor
3. Add ROS2 publisher in `arduino_bridge.py`
4. Update `arduino_params.yaml` with sensor config
5. Add sensor to URDF in `description/`

## Safety Features

- **Watchdog Timer**: Stops robot if no cmd_vel received for 1 second
- **Velocity Clamping**: Limits velocities to safe ranges
- **Connection Monitoring**: Detects and recovers from serial disconnections
- **Emergency Stop**: Arduino firmware includes emergency stop functionality

## Performance Tuning

### Reduce Latency

- Increase `encoder_read_rate` in config (default: 10 Hz)
- Decrease `timeout` in config (default: 1.0 s)

### Improve Accuracy

- Calibrate `wheel_separation` by driving in circles
- Calibrate `wheel_radius` by driving straight distances
- Verify `encoder_ppr` matches Hall sensor specifications

### Optimize CPU Usage

- Decrease `publish_rate` if not needed (default: 50 Hz)
- Disable RViz if not visualizing: `use_rviz:=false`

## References

- **Arduino Firmware**: `/WS/Arduino-Robot-control/MOTOR-INTERFACE-V-13/`
- **ROS2 Documentation**: https://docs.ros.org/en/jazzy/
- **Serial Protocol**: See Arduino's `ROS2_Bridge.h`

## Support

For issues or questions:
1. Check this README
2. Test Arduino connection with test script
3. Check ROS2 logs: `ros2 wtf`
4. Review Arduino Serial Monitor output

---

**Package**: smart_t_ai_v2  
**Version**: 1.0.0  
**ROS2 Distro**: Jazzy  
**License**: Apache-2.0
