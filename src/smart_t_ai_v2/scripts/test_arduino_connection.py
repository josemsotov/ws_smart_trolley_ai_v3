#!/usr/bin/env python3
"""
Test script for Arduino connection and basic communication.
This script helps verify that the Arduino is properly connected and responding.
"""

import serial
import time
import sys

def test_arduino_connection(port='/dev/ttyACM0', baud=115200):
    """Test basic Arduino connection and commands."""
    
    print(f"\n{'='*60}")
    print("SMART TROLLEY - Arduino Connection Test")
    print(f"{'='*60}\n")
    
    print(f"Attempting to connect to Arduino on {port} at {baud} baud...")
    
    try:
        # Open serial connection
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
        
        print("✓ Connection established!\n")
        
        # Clear any initial data
        ser.reset_input_buffer()
        
        # Test 1: Status command
        print("Test 1: Sending status command 's'...")
        ser.write(b's\n')
        time.sleep(0.1)
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  Response: {response if response else 'No response'}")
        
        # Test 2: Encoder reading
        print("\nTest 2: Reading encoders 'e'...")
        ser.write(b'e\n')
        time.sleep(0.1)
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  Response: {response if response else 'No response'}")
        
        if response.startswith('e'):
            try:
                parts = response.split()
                left = int(parts[1])
                right = int(parts[2])
                print(f"  ✓ Left encoder: {left}, Right encoder: {right}")
            except:
                print(f"  ⚠ Could not parse encoder values")
        
        # Test 3: Send velocity command (zero velocity - safe)
        print("\nTest 3: Sending zero velocity command 'v 0.0 0.0'...")
        ser.write(b'v 0.0 0.0\n')
        time.sleep(0.1)
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  Response: {response if response else 'OK (no response expected)'}")
        
        # Test 4: Monitor incoming data for 3 seconds
        print("\nTest 4: Monitoring serial data for 3 seconds...")
        start_time = time.time()
        message_count = 0
        while time.time() - start_time < 3:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    print(f"  << {data}")
                    message_count += 1
            time.sleep(0.1)
        
        if message_count == 0:
            print("  No spontaneous messages received (this is normal)")
        
        # Close connection
        ser.close()
        
        print(f"\n{'='*60}")
        print("✓ All tests completed successfully!")
        print(f"{'='*60}\n")
        print("Next steps:")
        print("  1. Build the package: colcon build --packages-select smart_t_ai_v2")
        print("  2. Source the workspace: source install/setup.bash")
        print("  3. Launch the robot: ros2 launch smart_t_ai_v2 launch_robot.launch.py")
        print("  4. Control with teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard")
        print()
        
        return True
        
    except serial.SerialException as e:
        print(f"\n✗ Serial connection error: {e}")
        print("\nTroubleshooting:")
        print("  1. Check if Arduino is connected: ls /dev/ttyACM* /dev/ttyUSB*")
        print("  2. Check permissions: sudo usermod -a -G dialout $USER")
        print("  3. Logout and login again for group changes to take effect")
        print("  4. Try a different port (e.g., /dev/ttyUSB0)")
        print()
        return False
        
    except KeyboardInterrupt:
        print("\n\n✗ Test interrupted by user")
        return False
        
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == '__main__':
    # Default port
    port = '/dev/ttyACM0'
    
    # Check for command line argument
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print("\nUsage: python3 test_arduino_connection.py [port]")
    print(f"Using port: {port}\n")
    
    # Check if pyserial is installed
    try:
        import serial
    except ImportError:
        print("✗ Error: pyserial not installed")
        print("Install it with: pip3 install pyserial")
        sys.exit(1)
    
    # Run test
    success = test_arduino_connection(port)
    sys.exit(0 if success else 1)
