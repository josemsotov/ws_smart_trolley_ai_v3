#!/usr/bin/env python3
"""
Person Follower Node - Smart Trolley V5
Sigue a una persona usando el Kinect (depth camera).
Mantiene a la persona en cuadro y a una distancia objetivo.

Author: Smart Trolley Team
Date: 2026-02-24
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2


class PersonFollower(Node):
    """
    Nodo que sigue a una persona usando la cámara de profundidad del Kinect.
    Controla velocidad lineal (distancia) y angular (centrado).
    """

    def __init__(self):
        super().__init__('person_follower')

        # ──────────────────────────────────────────────────────────
        # Parámetros configurables
        # ──────────────────────────────────────────────────────────
        self.declare_parameter('target_distance', 1.0)      # metros
        self.declare_parameter('min_distance', 0.6)         # metros
        self.declare_parameter('max_distance', 2.5)         # metros
        self.declare_parameter('max_linear_speed', 0.3)     # m/s
        self.declare_parameter('max_angular_speed', 0.8)    # rad/s
        self.declare_parameter('linear_kp', 0.5)            # Ganancia proporcional lineal
        self.declare_parameter('angular_kp', 1.2)           # Ganancia proporcional angular
        self.declare_parameter('depth_threshold', 0.1)      # metros (mínima prof. válida)
        self.declare_parameter('roi_width_ratio', 0.6)      # 60% del ancho central
        self.declare_parameter('roi_height_ratio', 0.7)     # 70% de la altura central
        self.declare_parameter('min_detection_area', 500)   # píxeles mínimos para detección válida
        self.declare_parameter('depth_topic', '/kinect/depth/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_debug', True)        # Publicar imagen debug
        
        self.target_distance = self.get_parameter('target_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.depth_threshold = self.get_parameter('depth_threshold').value
        self.roi_width_ratio = self.get_parameter('roi_width_ratio').value
        self.roi_height_ratio = self.get_parameter('roi_height_ratio').value
        self.min_detection_area = self.get_parameter('min_detection_area').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.enable_debug = self.get_parameter('enable_debug').value

        # ──────────────────────────────────────────────────────────
        # ROS2 Publishers & Subscribers
        # ──────────────────────────────────────────────────────────
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )
        
        if self.enable_debug:
            self.debug_image_pub = self.create_publisher(
                Image,
                '/person_follower/debug_image',
                10
            )

        # ──────────────────────────────────────────────────────────
        # Estado interno
        # ──────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.last_detection_time = self.get_clock().now()
        self.detection_timeout = 1.5  # segundos sin detección → stop
        
        # Timer para verificar timeout
        self.create_timer(0.1, self.check_timeout)
        
        self.get_logger().info('═══════════════════════════════════════════════════')
        self.get_logger().info('  PERSON FOLLOWER - Kinect Depth Tracking')
        self.get_logger().info('═══════════════════════════════════════════════════')
        self.get_logger().info(f'  Target distance: {self.target_distance:.2f} m')
        self.get_logger().info(f'  Distance range: [{self.min_distance:.2f}, {self.max_distance:.2f}] m')
        self.get_logger().info(f'  Max speeds: linear={self.max_linear_speed:.2f} m/s, angular={self.max_angular_speed:.2f} rad/s')
        self.get_logger().info(f'  PID gains: Kp_linear={self.linear_kp:.2f}, Kp_angular={self.angular_kp:.2f}')
        self.get_logger().info('═══════════════════════════════════════════════════')

    def depth_callback(self, msg):
        """
        Procesa imagen de profundidad del Kinect.
        Detecta la persona (objeto más cercano en ROI central),
        calcula distancia y offset lateral, y genera comandos de velocidad.
        """
        try:
            # Convertir ROS Image a numpy array
            # Kinect depth: 16-bit, valores en mm
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Convertir de mm a metros
            depth_image = depth_image.astype(np.float32) / 1000.0
            
            # Obtener dimensiones
            height, width = depth_image.shape
            
            # ─────────────────────────────────────────────────
            # Definir ROI (Region of Interest) central
            # ─────────────────────────────────────────────────
            roi_w = int(width * self.roi_width_ratio)
            roi_h = int(height * self.roi_height_ratio)
            roi_x = (width - roi_w) // 2
            roi_y = (height - roi_h) // 2
            
            roi = depth_image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
            
            # ─────────────────────────────────────────────────
            # Filtrar valores inválidos
            # ─────────────────────────────────────────────────
            # Eliminar valores 0 (sin lectura) y muy pequeños
            valid_mask = (roi > self.depth_threshold) & (roi < self.max_distance)
            
            if np.sum(valid_mask) < self.min_detection_area:
                # No hay suficientes píxeles válidos → persona no detectada
                self.get_logger().debug('No valid depth data in ROI')
                self.stop_robot()
                return
            
            # ─────────────────────────────────────────────────
            # Detectar la persona (objeto más cercano)
            # ─────────────────────────────────────────────────
            valid_depths = roi[valid_mask]
            min_depth = np.min(valid_depths)
            
            # Crear máscara de la persona (píxeles cercanos a la distancia mínima)
            depth_tolerance = 0.3  # metros
            person_mask = valid_mask & (roi < min_depth + depth_tolerance)
            
            if np.sum(person_mask) < self.min_detection_area:
                self.get_logger().debug('Person blob too small')
                self.stop_robot()
                return
            
            # ─────────────────────────────────────────────────
            # Calcular centroide de la persona
            # ─────────────────────────────────────────────────
            y_indices, x_indices = np.nonzero(person_mask)
            
            if len(x_indices) == 0:
                self.stop_robot()
                return
            
            # Centroide en coordenadas del ROI
            centroid_x = np.mean(x_indices)
            centroid_y = np.mean(y_indices)
            
            # Distancia promedio de la persona
            person_distance = np.mean(roi[person_mask])
            
            # ─────────────────────────────────────────────────
            # Calcular errores para control
            # ─────────────────────────────────────────────────
            # Error de distancia
            distance_error = person_distance - self.target_distance
            
            # Error lateral (offset del centro del ROI)
            roi_center_x = roi_w / 2.0
            lateral_error = centroid_x - roi_center_x
            lateral_error_normalized = lateral_error / roi_center_x  # [-1, 1]
            
            # ─────────────────────────────────────────────────
            # Control de velocidad
            # ─────────────────────────────────────────────────
            # Velocidad lineal: proporcional a error de distancia
            linear_vel = self.linear_kp * distance_error
            linear_vel = np.clip(linear_vel, -self.max_linear_speed, self.max_linear_speed)
            
            # No avanzar si está muy cerca
            if person_distance < self.min_distance:
                linear_vel = min(0.0, linear_vel)
            
            # Velocidad angular: proporcional a error lateral
            angular_vel = -self.angular_kp * lateral_error_normalized
            angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
            
            # ─────────────────────────────────────────────────
            # Publicar comando de velocidad
            # ─────────────────────────────────────────────────
            cmd = Twist()
            cmd.linear.x = float(linear_vel)
            cmd.angular.z = float(angular_vel)
            self.cmd_vel_pub.publish(cmd)
            
            # Actualizar tiempo de última detección
            self.last_detection_time = self.get_clock().now()
            
            # ─────────────────────────────────────────────────
            # Logging
            # ─────────────────────────────────────────────────
            self.get_logger().info(
                f'Person detected | Distance: {person_distance:.2f}m '
                f'(target: {self.target_distance:.2f}m) | '
                f'Lateral offset: {lateral_error_normalized:+.2f} | '
                f'Vel: lin={linear_vel:.2f} m/s, ang={angular_vel:.2f} rad/s'
            )
            
            # ─────────────────────────────────────────────────
            # Imagen de debug (opcional)
            # ─────────────────────────────────────────────────
            if self.enable_debug:
                self.publish_debug_image(
                    depth_image, roi_x, roi_y, roi_w, roi_h,
                    person_mask, centroid_x, centroid_y, person_distance
                )
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {e}')
            self.stop_robot()

    def publish_debug_image(self, depth_image, roi_x, roi_y, roi_w, roi_h,
                           person_mask, centroid_x, centroid_y, distance):
        """
        Genera y publica imagen de debug mostrando la detección.
        """
        try:
            # Normalizar depth para visualización (0-255)
            depth_vis = np.clip(depth_image * 255.0 / 3.0, 0, 255).astype(np.uint8)
            
            # Convertir a BGR para dibujar colores
            debug_img = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)
            
            # Dibujar ROI
            cv2.rectangle(debug_img, 
                         (roi_x, roi_y), 
                         (roi_x + roi_w, roi_y + roi_h),
                         (0, 255, 0), 2)
            
            # Dibujar máscara de persona en ROI
            roi_img = debug_img[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
            roi_img[person_mask] = [0, 0, 255]  # Rojo
            
            # Dibujar centroide
            centroid_abs_x = int(roi_x + centroid_x)
            centroid_abs_y = int(roi_y + centroid_y)
            cv2.circle(debug_img, (centroid_abs_x, centroid_abs_y), 10, (255, 0, 0), -1)
            
            # Dibujar línea del centro a centroide
            center_x = debug_img.shape[1] // 2
            center_y = debug_img.shape[0] // 2
            cv2.line(debug_img, (center_x, center_y), 
                    (centroid_abs_x, centroid_abs_y), (255, 255, 0), 2)
            
            # Añadir texto con distancia
            text = f'Distance: {distance:.2f}m'
            cv2.putText(debug_img, text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Publicar
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = 'kinect_depth_optical'
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Error publishing debug image: {e}')

    def stop_robot(self):
        """
        Detiene el robot publicando velocidad cero.
        """
        try:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
        except Exception:
            pass  # Contexto RCL ya invalidado en shutdown

    def check_timeout(self):
        """
        Verifica si ha pasado demasiado tiempo sin detectar persona.
        Si es así, detiene el robot.
        """
        time_since_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        
        if time_since_detection > self.detection_timeout:
            self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Person Follower interrupted by user')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
