#!/usr/bin/env python3
"""
ROS2 Node: Coral EdgeTPU Object Detector for Smart Trolley V5.

Subscribes to /kinect/rgb/image_raw (or configurable topic),
runs SSD MobileNet v2 on the Google Coral USB Accelerator,
and publishes detection results.

Topics published:
  /coral/detections  (std_msgs/String)  - JSON with detections
  /coral/image_detections (sensor_msgs/Image) - annotated image (optional)

Requirements:
  - ai-edge-litert==1.3.0
  - libedgetpu1-std (16.0tf2.18.1, feranick build)
  - Model: ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite
  - Labels: coco_labels.txt

Author: Smart Trolley V5 Project
"""

import os
import json
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import cv2

# ai-edge-litert (successor to tflite-runtime)
from ai_edge_litert.interpreter import Interpreter, load_delegate


class CoralDetectorNode(Node):
    """Object detection node using Google Coral EdgeTPU."""

    def __init__(self):
        super().__init__('coral_detector')

        # ─── Parameters ───
        self.declare_parameter('model_path', '')
        self.declare_parameter('labels_path', '')
        self.declare_parameter('image_topic', '/kinect/rgb/image_raw')
        self.declare_parameter('score_threshold', 0.4)
        self.declare_parameter('max_detections', 10)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('publish_annotated', False)
        self.declare_parameter('use_edgetpu', True)

        model_path = self.get_parameter('model_path').value
        labels_path = self.get_parameter('labels_path').value
        self.image_topic = self.get_parameter('image_topic').value
        self.score_threshold = self.get_parameter('score_threshold').value
        self.max_detections = self.get_parameter('max_detections').value
        publish_rate = self.get_parameter('publish_rate').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.use_edgetpu = self.get_parameter('use_edgetpu').value

        # ─── Resolve default paths ───
        if not model_path:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('smart_t_ai_v2')
            model_path = os.path.join(pkg_dir, 'models',
                                      'ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite')
        if not labels_path:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('smart_t_ai_v2')
            labels_path = os.path.join(pkg_dir, 'models', 'coco_labels.txt')

        # ─── Load labels ───
        self.labels = {}
        if os.path.exists(labels_path):
            with open(labels_path, 'r') as f:
                self.labels = {i: line.strip() for i, line in enumerate(f.readlines())}
            self.get_logger().info(f'Loaded {len(self.labels)} labels from {labels_path}')
        else:
            self.get_logger().warn(f'Labels file not found: {labels_path}')

        # ─── Load TFLite model with EdgeTPU delegate ───
        self.interpreter = None
        try:
            if self.use_edgetpu:
                self.get_logger().info('Loading EdgeTPU delegate...')
                delegate = load_delegate('libedgetpu.so.1')
                self.interpreter = Interpreter(
                    model_path=model_path,
                    experimental_delegates=[delegate]
                )
                self.get_logger().info('✅ EdgeTPU delegate loaded')
            else:
                self.interpreter = Interpreter(model_path=model_path)
                self.get_logger().info('Running on CPU (no EdgeTPU)')

            self.interpreter.allocate_tensors()
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()

            self.input_shape = self.input_details[0]['shape']  # [1, H, W, 3]
            self.input_h = self.input_shape[1]
            self.input_w = self.input_shape[2]

            self.get_logger().info(
                f'Model loaded: input={self.input_shape}, '
                f'outputs={len(self.output_details)}'
            )

            # Warmup inference
            dummy = np.zeros(self.input_shape, dtype=np.uint8)
            self.interpreter.set_tensor(self.input_details[0]['index'], dummy)
            self.interpreter.invoke()
            self.get_logger().info('Warmup inference completed')

        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.interpreter = None

        # ─── Image state ───
        self._lock = threading.Lock()
        self._latest_image = None
        self._latest_stamp = None
        self._frame_count = 0
        self._detect_count = 0
        self._last_stats_time = time.time()

        # ─── Publishers ───
        self.detection_pub = self.create_publisher(String, '/coral/detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/coral/image_detections', 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/coral/markers', 10)

        # ─── Color palette for bounding boxes (RGB order) ───
        self.COLORS = [
            (0, 255, 0), (255, 50, 50), (50, 50, 255), (255, 255, 0),
            (255, 0, 255), (0, 255, 255), (128, 255, 0), (255, 128, 0),
            (0, 128, 255), (255, 0, 128), (128, 0, 255), (0, 255, 128),
        ]

        # ─── Depth state (for 3D markers) ───
        self._latest_depth = None
        self._fx = 525.0   # Kinect default intrinsics
        self._fy = 525.0
        self._cx = 319.5
        self._cy = 239.5

        # ─── Subscribers ───
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, 1
        )
        self.depth_sub = self.create_subscription(
            Image, '/kinect/depth/image_raw', self._depth_callback, 1
        )
        self.caminfo_sub = self.create_subscription(
            CameraInfo, '/kinect/rgb/camera_info', self._caminfo_callback, 1
        )

        # ─── Detection timer ───
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self._detect_callback)

        # ─── Stats timer (every 10s) ───
        self.stats_timer = self.create_timer(10.0, self._stats_callback)

        self.get_logger().info(
            f'Coral detector started: topic={self.image_topic}, '
            f'rate={publish_rate}Hz, threshold={self.score_threshold}, '
            f'edgetpu={self.use_edgetpu}'
        )

    def _image_callback(self, msg: Image):
        """Store latest image for detection."""
        with self._lock:
            self._latest_image = msg
            self._latest_stamp = msg.header.stamp

    def _depth_callback(self, msg: Image):
        """Store latest depth image for 3D marker positioning."""
        self._latest_depth = msg

    def _caminfo_callback(self, msg: CameraInfo):
        """Update camera intrinsics from camera_info (once)."""
        if msg.k[0] > 0:
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]
            self.destroy_subscription(self.caminfo_sub)
            self.get_logger().info(
                f'Camera intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
                f'cx={self._cx:.1f} cy={self._cy:.1f}'
            )

    def _detect_callback(self):
        """Run detection on latest image."""
        if self.interpreter is None:
            return

        with self._lock:
            if self._latest_image is None:
                return
            img_msg = self._latest_image
            self._latest_image = None  # consume

        self._frame_count += 1

        try:
            # ─── Convert ROS Image to numpy ───
            h = img_msg.height
            w = img_msg.width
            encoding = img_msg.encoding

            if encoding in ('rgb8', 'RGB8'):
                img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(h, w, 3)
            elif encoding in ('bgr8', 'BGR8'):
                img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(h, w, 3)
                img_np = img_np[:, :, ::-1]  # BGR → RGB
            else:
                self.get_logger().warn(f'Unsupported encoding: {encoding}', throttle_duration_sec=5.0)
                return

            # ─── Resize to model input ───
            # Simple nearest-neighbor resize (fast, no cv2 dependency)
            if h != self.input_h or w != self.input_w:
                row_idx = (np.arange(self.input_h) * h / self.input_h).astype(int)
                col_idx = (np.arange(self.input_w) * w / self.input_w).astype(int)
                img_resized = img_np[row_idx][:, col_idx]
            else:
                img_resized = img_np

            input_data = np.expand_dims(img_resized, axis=0).astype(np.uint8)

            # ─── Inference ───
            t0 = time.time()
            self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
            self.interpreter.invoke()
            inference_ms = (time.time() - t0) * 1000

            # ─── Parse outputs (SSD MobileNet format) ───
            # Output 0: boxes [1, N, 4] (ymin, xmin, ymax, xmax) normalized
            # Output 1: classes [1, N]
            # Output 2: scores [1, N]
            # Output 3: count [1]
            boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
            classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
            scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]
            count = int(self.interpreter.get_tensor(self.output_details[3]['index'])[0])

            # ─── Filter detections ───
            detections = []
            for i in range(min(count, self.max_detections)):
                if scores[i] >= self.score_threshold:
                    class_id = int(classes[i])
                    label = self.labels.get(class_id, f'class_{class_id}')
                    ymin, xmin, ymax, xmax = boxes[i]
                    detections.append({
                        'label': label,
                        'class_id': class_id,
                        'score': round(float(scores[i]), 3),
                        'bbox': {
                            'xmin': round(float(xmin) * w, 1),
                            'ymin': round(float(ymin) * h, 1),
                            'xmax': round(float(xmax) * w, 1),
                            'ymax': round(float(ymax) * h, 1),
                        }
                    })

            self._detect_count += len(detections)

            # ─── Publish detections as JSON ───
            msg = String()
            msg.data = json.dumps({
                'stamp': {
                    'sec': img_msg.header.stamp.sec,
                    'nanosec': img_msg.header.stamp.nanosec,
                },
                'inference_ms': round(inference_ms, 1),
                'num_detections': len(detections),
                'detections': detections,
            })
            self.detection_pub.publish(msg)

            # ─── Publish annotated image with bounding boxes ───
            if self.publish_annotated:
                annotated = img_np.copy()
                for det in detections:
                    bbox = det['bbox']
                    x1 = int(bbox['xmin'])
                    y1 = int(bbox['ymin'])
                    x2 = int(bbox['xmax'])
                    y2 = int(bbox['ymax'])
                    color = self.COLORS[det['class_id'] % len(self.COLORS)]

                    # Bounding box
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

                    # Label with solid background
                    label_text = f"{det['label']} {det['score']:.0%}"
                    (tw, th), _ = cv2.getTextSize(
                        label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    ly = max(y1 - th - 10, 0)
                    cv2.rectangle(annotated,
                                  (x1, ly), (x1 + tw + 4, y1),
                                  color, -1)
                    cv2.putText(annotated, label_text,
                                (x1 + 2, max(y1 - 5, th + 2)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 0, 0), 2)

                # Inference stats overlay
                cv2.putText(annotated,
                            f'{inference_ms:.0f}ms | {len(detections)} det',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 255, 255), 2)

                ann_msg = Image()
                ann_msg.header = img_msg.header
                ann_msg.height = h
                ann_msg.width = w
                ann_msg.encoding = 'rgb8'
                ann_msg.step = w * 3
                ann_msg.data = annotated.tobytes()
                self.annotated_pub.publish(ann_msg)

            # ─── Publish 3D markers for RViz ───
            marker_array = MarkerArray()

            # Clear previous markers
            clear = Marker()
            clear.action = Marker.DELETEALL
            clear.header.frame_id = 'kinect_rgb_optical'
            clear.header.stamp = img_msg.header.stamp
            marker_array.markers.append(clear)

            depth_img = self._latest_depth  # atomic read

            for i, det in enumerate(detections):
                bbox = det['bbox']
                cx_px = (bbox['xmin'] + bbox['xmax']) / 2.0
                cy_px = (bbox['ymin'] + bbox['ymax']) / 2.0

                # ── Get depth at detection center ──
                depth_m = 2.0  # fallback
                if depth_img is not None:
                    try:
                        dh = depth_img.height
                        dw = depth_img.width
                        dx = int(min(max(cx_px * dw / w, 0), dw - 1))
                        dy = int(min(max(cy_px * dh / h, 0), dh - 1))

                        if depth_img.encoding in ('16UC1', 'mono16'):
                            d_arr = np.frombuffer(
                                depth_img.data, dtype=np.uint16
                            ).reshape(dh, dw)
                            ys = max(0, dy - 2)
                            ye = min(dh, dy + 3)
                            xs = max(0, dx - 2)
                            xe = min(dw, dx + 3)
                            region = d_arr[ys:ye, xs:xe]
                            valid = region[region > 0]
                            if len(valid) > 0:
                                depth_m = float(np.median(valid)) / 1000.0
                        elif depth_img.encoding == '32FC1':
                            d_arr = np.frombuffer(
                                depth_img.data, dtype=np.float32
                            ).reshape(dh, dw)
                            d = d_arr[dy, dx]
                            if d > 0 and np.isfinite(d):
                                depth_m = float(d)
                    except Exception:
                        pass

                # ── Pixel → 3D (optical frame: Z forward) ──
                x3d = (cx_px - self._cx) * depth_m / self._fx
                y3d = (cy_px - self._cy) * depth_m / self._fy
                z3d = depth_m

                color = self.COLORS[det['class_id'] % len(self.COLORS)]
                r_f = color[0] / 255.0
                g_f = color[1] / 255.0
                b_f = color[2] / 255.0

                # ── Text label floating above ──
                t = Marker()
                t.header.frame_id = 'kinect_rgb_optical'
                t.header.stamp = img_msg.header.stamp
                t.ns = 'coral_labels'
                t.id = i
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = x3d
                t.pose.position.y = y3d - 0.15
                t.pose.position.z = z3d
                t.pose.orientation.w = 1.0
                t.scale.z = 0.12
                t.color.r = r_f
                t.color.g = g_f
                t.color.b = b_f
                t.color.a = 1.0
                t.text = f"{det['label']} {det['score']:.0%}"
                t.lifetime = Duration(sec=0, nanosec=500000000)
                marker_array.markers.append(t)

                # ── Semi-transparent bounding cube ──
                bw = abs(bbox['xmax'] - bbox['xmin']) * depth_m / self._fx
                bh = abs(bbox['ymax'] - bbox['ymin']) * depth_m / self._fy
                c = Marker()
                c.header.frame_id = 'kinect_rgb_optical'
                c.header.stamp = img_msg.header.stamp
                c.ns = 'coral_boxes'
                c.id = i
                c.type = Marker.CUBE
                c.action = Marker.ADD
                c.pose.position.x = x3d
                c.pose.position.y = y3d
                c.pose.position.z = z3d
                c.pose.orientation.w = 1.0
                c.scale.x = bw
                c.scale.y = bh
                c.scale.z = 0.05
                c.color.r = r_f
                c.color.g = g_f
                c.color.b = b_f
                c.color.a = 0.3
                c.lifetime = Duration(sec=0, nanosec=500000000)
                marker_array.markers.append(c)

            self.marker_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}', throttle_duration_sec=5.0)

    def _stats_callback(self):
        """Log detection statistics."""
        now = time.time()
        elapsed = now - self._last_stats_time
        if elapsed > 0:
            fps = self._frame_count / elapsed
            self.get_logger().info(
                f'Coral: {fps:.1f} FPS, '
                f'{self._detect_count} detections in {self._frame_count} frames'
            )
        self._frame_count = 0
        self._detect_count = 0
        self._last_stats_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CoralDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
