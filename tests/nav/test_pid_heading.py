#!/usr/bin/env python3
"""
Test PID Heading Control
Envía velocidad lineal durante X segundos y mide la desviación lateral.
Compara el resultado con/sin PID.

Uso:
  python3 test_pid_heading.py          # Test con PID (default)
  python3 test_pid_heading.py --no-pid # Test sin PID (para comparar)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import sys
import signal


class PIDHeadingTest(Node):
    def __init__(self, disable_pid=False):
        super().__init__('pid_heading_test')
        
        self.disable_pid = disable_pid
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False
        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None
        self.max_lateral_dev = 0.0
        self.max_heading_dev = 0.0
        self.samples = []
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # If disabling PID, set parameter
        if self.disable_pid:
            self.get_logger().info('⚠️  Deshabilitando PID heading para comparación...')
            import subprocess
            subprocess.run([
                'ros2', 'param', 'set', '/arduino_bridge', 
                'pid_heading_enabled', 'false'
            ], capture_output=True)
        else:
            import subprocess
            subprocess.run([
                'ros2', 'param', 'set', '/arduino_bridge', 
                'pid_heading_enabled', 'true'
            ], capture_output=True)
    
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract theta from quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = 2.0 * math.atan2(qz, qw)
        self.odom_received = True
    
    def send_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)
    
    def stop(self):
        for _ in range(5):
            self.send_vel(0.0, 0.0)
            time.sleep(0.05)
    
    def wait_for_odom(self, timeout=5.0):
        """Wait until we receive odometry."""
        start = time.time()
        while not self.odom_received and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom_received
    
    def run_test(self, velocity=0.3, duration=5.0):
        """Run the straight-line test."""
        mode = "SIN PID" if self.disable_pid else "CON PID"
        
        print(f"\n{'='*60}")
        print(f"  TEST DE DIRECCIÓN - {mode}")
        print(f"  Velocidad: {velocity} m/s | Duración: {duration}s")
        print(f"{'='*60}\n")
        
        # Wait for odometry
        print("⏳ Esperando odometría...")
        if not self.wait_for_odom():
            print("❌ No se recibe odometría. ¿Está arduino_bridge corriendo?")
            return
        
        # Record initial position
        self.initial_x = self.x
        self.initial_y = self.y
        self.initial_theta = self.theta
        
        print(f"📍 Posición inicial: x={self.initial_x:.3f}, y={self.initial_y:.3f}")
        print(f"🧭 Heading inicial:  {math.degrees(self.initial_theta):.1f}°")
        print()
        
        # Start moving
        print(f"🚀 Avanzando a {velocity} m/s...")
        print(f"{'t(s)':>5} | {'dist(m)':>8} | {'lat_dev(m)':>10} | {'heading(°)':>10} | {'h_error(°)':>10}")
        print(f"{'-'*5}-+-{'-'*8}-+-{'-'*10}-+-{'-'*10}-+-{'-'*10}")
        
        start_time = time.time()
        last_print = 0
        
        while time.time() - start_time < duration:
            self.send_vel(velocity, 0.0)
            # Spin multiple times to catch odom updates (published at 50Hz)
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.01)
            
            elapsed = time.time() - start_time
            
            # Print every 0.5 seconds
            if elapsed - last_print >= 0.5:
                last_print = elapsed
                
                # Calculate forward distance along initial heading
                dx = self.x - self.initial_x
                dy = self.y - self.initial_y
                
                # Project onto initial heading direction
                cos_h = math.cos(self.initial_theta)
                sin_h = math.sin(self.initial_theta)
                forward_dist = dx * cos_h + dy * sin_h
                lateral_dev = -dx * sin_h + dy * cos_h
                
                heading_error = math.degrees(
                    math.atan2(
                        math.sin(self.theta - self.initial_theta),
                        math.cos(self.theta - self.initial_theta)
                    )
                )
                
                self.max_lateral_dev = max(self.max_lateral_dev, abs(lateral_dev))
                self.max_heading_dev = max(self.max_heading_dev, abs(heading_error))
                
                self.samples.append({
                    't': elapsed,
                    'dist': forward_dist,
                    'lat': lateral_dev,
                    'heading': heading_error
                })
                
                print(f"{elapsed:5.1f} | {forward_dist:8.3f} | {lateral_dev:10.3f} | "
                      f"{math.degrees(self.theta):10.1f} | {heading_error:10.2f}")
        
        # Stop
        self.stop()
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.2)
        
        # Final metrics
        dx = self.x - self.initial_x
        dy = self.y - self.initial_y
        cos_h = math.cos(self.initial_theta)
        sin_h = math.sin(self.initial_theta)
        total_forward = dx * cos_h + dy * sin_h
        final_lateral = -dx * sin_h + dy * cos_h
        final_heading_error = math.degrees(
            math.atan2(
                math.sin(self.theta - self.initial_theta),
                math.cos(self.theta - self.initial_theta)
            )
        )
        
        print(f"\n{'='*60}")
        print(f"  RESULTADOS - {mode}")
        print(f"{'='*60}")
        print(f"  Distancia recorrida:      {total_forward:.3f} m")
        print(f"  Desviación lateral final: {final_lateral:.3f} m")
        print(f"  Desviación lateral máx:   {self.max_lateral_dev:.3f} m")
        print(f"  Error heading final:      {final_heading_error:.2f}°")
        print(f"  Error heading máximo:     {self.max_heading_dev:.2f}°")
        print(f"  % lateral vs distancia:   {abs(final_lateral/max(total_forward,0.01))*100:.1f}%")
        print(f"{'='*60}\n")
        
        # Re-enable PID if we disabled it
        if self.disable_pid:
            import subprocess
            subprocess.run([
                'ros2', 'param', 'set', '/arduino_bridge', 
                'pid_heading_enabled', 'true'
            ], capture_output=True)
            print("✅ PID heading re-habilitado\n")


def main():
    rclpy.init()
    
    disable_pid = '--no-pid' in sys.argv
    
    node = PIDHeadingTest(disable_pid=disable_pid)
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\n⚠️  Interrumpido - deteniendo robot...")
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        node.run_test(velocity=0.3, duration=5.0)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
