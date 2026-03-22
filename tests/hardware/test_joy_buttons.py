#!/usr/bin/env python3
"""Monitor joystick input in real-time with visual feedback"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import os

class JoyMonitor(Node):
    def __init__(self):
        super().__init__('joy_monitor')
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.prev_buttons = []
        self.prev_axes = []
        
        # Clear screen
        os.system('clear')
        print("=" * 60)
        print(" 🎮 MONITOR DE CONTROL - Presiona botones y mueve sticks")
        print("=" * 60)
        print("\nEsperando datos del joystick...\n")
    
    def joy_callback(self, msg):
        # Detect button changes
        button_changes = []
        for i, btn in enumerate(msg.buttons):
            if i >= len(self.prev_buttons) or btn != self.prev_buttons[i]:
                if btn == 1:
                    button_changes.append(f"🔴 BOTÓN {i} PRESIONADO")
        
        # Detect axis changes (significant movement)
        axis_changes = []
        for i, axis in enumerate(msg.axes):
            if i >= len(self.prev_axes) or abs(axis - self.prev_axes[i]) > 0.1:
                if abs(axis) > 0.1:  # Ignore noise
                    direction = "+" if axis > 0 else "-"
                    axis_changes.append(f"🕹️  EJE {i}: {direction}{abs(axis):.2f}")
        
        # Print changes
        if button_changes or axis_changes:
            os.system('clear')
            print("=" * 60)
            print(" 🎮 MONITOR DE CONTROL")
            print("=" * 60)
            print()
            
            if button_changes:
                print("BOTONES ACTIVOS:")
                for change in button_changes:
                    print(f"  {change}")
                print()
            
            if axis_changes:
                print("EJES EN MOVIMIENTO:")
                for change in axis_changes:
                    print(f"  {change}")
                print()
            
            print("-" * 60)
            print("Identificados:")
            print(f"  • Total botones: {len(msg.buttons)}")
            print(f"  • Total ejes: {len(msg.axes)}")
            print()
            print("💡 Anota qué botón/eje corresponde a:")
            print("   - Botón de hombro izquierdo (L1/LB)")
            print("   - Stick izquierdo (arriba/abajo y izq/der)")
            print()
        
        # Save previous state
        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)

def main(args=None):
    rclpy.init(args=args)
    monitor = JoyMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n✓ Monitor cerrado")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
