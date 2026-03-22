#!/usr/bin/env python3
"""
Script de prueba simple para mover el robot sin gamepad
Presiona teclas para controlar:
  W = Avanzar
  S = Retroceder
  A = Girar izquierda
  D = Girar derecha
  SPACE = Detener
  Q = Salir
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('🎮 Controlador de teclado iniciado')
        self.get_logger().info('Usa: W=avanzar, S=retroceder, A=girar izq, D=girar der, SPACE=parar, Q=salir')
        
    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        self.get_logger().info(f'📤 Enviado: linear={linear:.2f}, angular={angular:.2f}')

def get_key():
    """Obtener tecla presionada sin esperar Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rclpy.init()
    controller = KeyboardController()
    
    linear_speed = 0.3  # m/s
    angular_speed = 0.8  # rad/s
    
    print('')
    print('=' * 50)
    print('  🎮 CONTROL POR TECLADO')
    print('=' * 50)
    print('')
    print('  W = Avanzar')
    print('  S = Retroceder')
    print('  A = Girar izquierda')
    print('  D = Girar derecha')
    print('  SPACE = Detener')
    print('  Q = Salir')
    print('')
    print('=' * 50)
    print('')
    
    try:
        while True:
            key = get_key().lower()
            
            if key == 'q':
                controller.send_velocity(0.0, 0.0)
                print('\n👋 Saliendo...')
                break
            elif key == 'w':
                controller.send_velocity(linear_speed, 0.0)
            elif key == 's':
                controller.send_velocity(-linear_speed, 0.0)
            elif key == 'a':
                controller.send_velocity(0.0, angular_speed)
            elif key == 'd':
                controller.send_velocity(0.0, -angular_speed)
            elif key == ' ':
                controller.send_velocity(0.0, 0.0)
            
            rclpy.spin_once(controller, timeout_sec=0.01)
            
    except KeyboardInterrupt:
        print('\n👋 Interrumpido por usuario')
    finally:
        controller.send_velocity(0.0, 0.0)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
