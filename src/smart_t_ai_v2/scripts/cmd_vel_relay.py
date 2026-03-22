#!/usr/bin/env python3
"""
cmd_vel_relay.py — Smart Trolley V5
Relay de /cmd_vel → /diff_drive_controller/cmd_vel_unstamped

El diff_drive_controller de ros2_control suscribe a su propio topic
~/cmd_vel_unstamped (/diff_drive_controller/cmd_vel_unstamped).
Este nodo reenvía cualquier Twist publicado en /cmd_vel al topic
que escucha el controlador, manteniendo /cmd_vel como interfaz
estándar para todos los nodos de control (teleop, person_follower,
test scripts, etc.).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._pub = self.create_publisher(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped',
            qos,
        )
        self._sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._relay,
            qos,
        )
        self.get_logger().info(
            'cmd_vel_relay: /cmd_vel → /diff_drive_controller/cmd_vel_unstamped'
        )

    def _relay(self, msg: Twist):
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
