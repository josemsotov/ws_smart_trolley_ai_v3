#!/usr/bin/env python3
"""
Test de movimiento: 5m adelante + 5m atrás
Monitorea odometría en tiempo real y detiene al alcanzar la distancia objetivo.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import sys


class MovementTest(Node):
    def __init__(self):
        super().__init__('movement_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False
        self.start_x = 0.0
        self.start_y = 0.0

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_received = True

    def distance_from_start(self):
        return math.sqrt((self.x - self.start_x)**2 + (self.y - self.start_y)**2)

    def send_vel(self, linear, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.send_vel(0.0)
            time.sleep(0.05)

    def wait_for_odom(self, timeout=5.0):
        t0 = time.time()
        while not self.odom_received and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom_received

    def move_distance(self, target_dist, speed):
        """Mover una distancia objetivo a la velocidad dada (+ adelante, - atrás)"""
        self.start_x = self.x
        self.start_y = self.y
        direction = "ADELANTE" if speed > 0 else "ATRÁS"

        self.get_logger().info(f'=== Moviendo {target_dist:.1f}m {direction} a {abs(speed):.2f} m/s ===')

        t0 = time.time()
        timeout = target_dist / abs(speed) * 3  # 3x el tiempo esperado como seguridad
        rate = 10  # Hz

        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            dist = self.distance_from_start()

            # Reportar cada 0.5 segundos
            elapsed = time.time() - t0
            if int(elapsed * rate) % (rate // 2) == 0:
                self.get_logger().info(
                    f'  Distancia: {dist:.3f}m / {target_dist:.1f}m  |  '
                    f'Pos: ({self.x:.3f}, {self.y:.3f})  |  '
                    f'Tiempo: {elapsed:.1f}s')

            if dist >= target_dist:
                self.stop()
                self.get_logger().info(
                    f'  *** OBJETIVO ALCANZADO: {dist:.3f}m en {elapsed:.1f}s ***')
                return True

            if elapsed > timeout:
                self.stop()
                self.get_logger().warn(
                    f'  *** TIMEOUT en {elapsed:.1f}s, distancia: {dist:.3f}m ***')
                return False

            self.send_vel(speed)
            time.sleep(1.0 / rate)

    def run_test(self):
        self.get_logger().info('='*60)
        self.get_logger().info('  TEST DE MOVIMIENTO: 5m ADELANTE + 5m ATRÁS')
        self.get_logger().info('='*60)

        # Esperar odometría
        if not self.wait_for_odom():
            self.get_logger().error('No se recibió odometría. Abortando.')
            return

        self.get_logger().info(f'Posición inicial: ({self.x:.3f}, {self.y:.3f})')
        self.get_logger().info('Iniciando en 3 segundos...')
        time.sleep(3)

        # FASE 1: 5m adelante a 0.3 m/s
        speed = 0.3
        ok1 = self.move_distance(5.0, speed)

        self.get_logger().info(f'Posición tras avance: ({self.x:.3f}, {self.y:.3f})')
        self.get_logger().info('Pausa de 3 segundos...')
        time.sleep(3)

        # FASE 2: 5m atrás a -0.3 m/s
        ok2 = self.move_distance(5.0, -speed)

        # Resumen
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('  RESUMEN DEL TEST')
        self.get_logger().info('='*60)
        self.get_logger().info(f'  Fase 1 (5m adelante): {"PASS" if ok1 else "FAIL"}')
        self.get_logger().info(f'  Fase 2 (5m atrás):    {"PASS" if ok2 else "FAIL"}')
        self.get_logger().info(f'  Posición final: ({self.x:.3f}, {self.y:.3f})')
        error = math.sqrt(self.x**2 + self.y**2)
        self.get_logger().info(f'  Error retorno al origen: {error:.3f}m')
        self.get_logger().info('='*60)

        self.stop()


def main():
    rclpy.init()
    node = MovementTest()
    try:
        node.run_test()
    except KeyboardInterrupt:
        node.stop()
        node.get_logger().info('Test cancelado por usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
