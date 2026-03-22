#!/usr/bin/env python3
"""
motor_simple_test.py — Smart Trolley V5
Prueba mínima: avanza 2s → para 1s → retrocede 2s → para.
Requiere arduino_bridge corriendo.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class SimpleMotorTest(Node):
    def __init__(self):
        super().__init__('simple_motor_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Esperando 2s para que arduino_bridge conecte...')
        self._timer = self.create_timer(2.0, self._run_once)
        self._done = False

    def _cmd(self, lin, ang, secs, label):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        end = time.time() + secs
        self.get_logger().info(f'▶ {label}  ({secs}s)')
        while time.time() < end:
            self.pub.publish(msg)
            time.sleep(0.05)

    def _run_once(self):
        if self._done:
            return
        self._done = True
        self._timer.cancel()

        self._cmd( 0.15, 0.0, 2.0, 'AVANCE    lin=+0.15 m/s')
        self._cmd( 0.0,  0.0, 1.0, 'PARADA')
        self._cmd(-0.15, 0.0, 2.0, 'RETROCESO lin=-0.15 m/s')
        self._cmd( 0.0,  0.0, 0.5, 'PARADA FINAL')

        self.get_logger().info('✅ Test completado — motores en reposo.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMotorTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            msg = Twist()
            node.pub.publish(msg)   # asegurar stop
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
