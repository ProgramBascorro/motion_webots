#!/usr/bin/env python3
"""YOLO detector node for ROS2 - Python implementation with GPU support."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from soccer_msgs.msg import BoundingBox, BoundingBoxes
import cv2
import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import time


class YoloDetectorNode(Node):
    """YOLO-based object detector for soccer robotics."""

    CLASS_NAMES = ['ball', 'goal post', 'robot', 'L-intersection', 'T-intersection', 'X-intersection']
    CLASS_COLORS = [
        (0, 255, 255),    # ball - yellow
        (0, 255, 0),      # goal post - green
        (255, 0, 0),      # robot - blue
        (255, 255, 0),    # L-intersection - cyan
        (255, 0, 255),    # T-intersection - magenta
        (0, 255, 255),    # X-intersection - yellow
    ]

    def __init__(self):
        super().__init__('yolo_detector')

        # Declare parameters
        self.declare_parameter('image_topic', '/robotis_op3/camera/image_raw')
        self.declare_parameter('ball_center_topic', '/vision/yolo/ball_center')
        self.declare_parameter('model_path', 'models/yolo.onnx')
        self.declare_parameter('use_gpu', False)
        self.declare_parameter('use_fp16', False)
        self.declare_parameter('dnn_backend', 'opencv')
        self.declare_parameter('dnn_target', 'cpu')
        self.declare_parameter('input_size', 640)
        self.declare_parameter('ball_confidence_threshold', 0.2)
        self.declare_parameter('goalpost_confidence_threshold', 0.2)
        self.declare_parameter('robot_confidence_threshold', 0.2)
        self.declare_parameter('intersection_confidence_threshold', 0.4)
        self.declare_parameter('nms_threshold', 0.5)
        self.declare_parameter('nms_score_threshold', 0.1)
        self.declare_parameter('publish_debug', False)
        self.declare_parameter('frame_skip', 1)

        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.ball_center_topic = self.get_parameter('ball_center_topic').value
        self.model_path = self.resolve_model_path(self.get_parameter('model_path').value)
        self.use_gpu = self.get_parameter('use_gpu').value
        self.use_fp16 = self.get_parameter('use_fp16').value
        self.dnn_backend = self.get_parameter('dnn_backend').value
        self.dnn_target = self.get_parameter('dnn_target').value
        self.input_size = self.get_parameter('input_size').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.nms_score_threshold = self.get_parameter('nms_score_threshold').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.frame_skip = max(1, self.get_parameter('frame_skip').value)

        # Class-specific thresholds
        self.class_thresholds = [
            self.get_parameter('ball_confidence_threshold').value,
            self.get_parameter('goalpost_confidence_threshold').value,
            self.get_parameter('robot_confidence_threshold').value,
            self.get_parameter('intersection_confidence_threshold').value,
            self.get_parameter('intersection_confidence_threshold').value,
            self.get_parameter('intersection_confidence_threshold').value,
        ]

        # Performance tracking
        self.frame_counter = 0
        self.frame_count = 0
        self.total_inference_ms = 0.0
        self.total_full_ms = 0.0
        self.last_log_time = self.get_clock().now()

        # Load model
        self.net = None
        self.model_loaded = self.load_model()

        # Publishers
        self.debug_pub = self.create_publisher(Image, '/vision/yolo/debug', 10)
        self.detections_pub = self.create_publisher(BoundingBoxes, '/vision/yolo/detections', 10)
        if self.ball_center_topic:
            self.ball_center_pub = self.create_publisher(PointStamped, self.ball_center_topic, 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info('YOLO detector initialized (Python - 2D only)')
        self.get_logger().info(f'  Image topic: {self.image_topic}')
        self.get_logger().info(f'  Model: {self.model_path}')
        self.get_logger().info(f'  OpenCV version: {cv2.__version__}')

    def resolve_model_path(self, model_path):
        """Resolve model path relative to package share directory."""
        if not model_path:
            return model_path
        path = Path(model_path)
        if path.is_absolute():
            return str(path)
        pkg_share = get_package_share_directory('op3_yolo_vision')
        return str(Path(pkg_share) / model_path)

    def load_model(self):
        """Load YOLO ONNX model."""
        if not self.model_path:
            self.get_logger().error('model_path is empty')
            return False

        if not Path(self.model_path).exists():
            self.get_logger().error(f'YOLO model not found: {self.model_path}')
            return False

        try:
            self.net = cv2.dnn.readNetFromONNX(self.model_path)

            # Configure backend/target
            backend_str = self.dnn_backend.lower()
            target_str = self.dnn_target.lower()

            # Auto-configure for GPU
            if self.use_gpu:
                if backend_str in ['', 'opencv', 'default']:
                    backend_str = 'cuda'
                if target_str in ['', 'cpu', 'default']:
                    target_str = 'cuda_fp16' if self.use_fp16 else 'cuda'

            # Set backend
            if backend_str == 'cuda':
                backend = cv2.dnn.DNN_BACKEND_CUDA
            else:
                backend = cv2.dnn.DNN_BACKEND_OPENCV

            # Set target
            if target_str == 'cuda':
                target = cv2.dnn.DNN_TARGET_CUDA
            elif target_str in ['cuda_fp16', 'fp16']:
                target = cv2.dnn.DNN_TARGET_CUDA_FP16
            else:
                target = cv2.dnn.DNN_TARGET_CPU

            try:
                self.net.setPreferableBackend(backend)
                self.net.setPreferableTarget(target)
                self.get_logger().info(f'  DNN backend: {backend_str}')
                self.get_logger().info(f'  DNN target: {target_str}')
            except cv2.error as e:
                self.get_logger().warn(f'Failed to set DNN backend/target: {e}')
                self.get_logger().warn('Falling back to CPU (OpenCV)')
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        except cv2.error as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.get_logger().error('OpenCV DNN ONNX parsing failed. Try re-exporting model or upgrade OpenCV.')
            return False

        return True

    def image_callback(self, msg):
        """Process incoming image."""
        if not self.model_loaded:
            return

        # Frame skipping
        self.frame_counter += 1
        if self.frame_counter % self.frame_skip != 0:
            return

        start_time = time.perf_counter()

        # Convert ROS Image to OpenCV
        try:
            bgr = self.ros_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        height, width = bgr.shape[:2]

        # Preprocess - letterbox to square
        max_dim = max(width, height)
        if width == height:
            processed = cv2.resize(bgr, (self.input_size, self.input_size))
        else:
            square = np.full((max_dim, max_dim, 3), 114, dtype=np.uint8)
            square[0:height, 0:width] = bgr
            processed = cv2.resize(square, (self.input_size, self.input_size))

        scale = max_dim / self.input_size

        # Create blob and run inference
        blob = cv2.dnn.blobFromImage(
            processed, 1.0 / 255.0, (self.input_size, self.input_size),
            swapRB=True, crop=False
        )

        self.net.setInput(blob)
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())

        inference_time = time.perf_counter()
        inference_ms = (inference_time - start_time) * 1000

        if not outputs:
            self.get_logger().warn('YOLO inference returned no outputs')
            return

        # Decode detections
        detections = self.decode_detections(outputs[0], scale, width, height)

        if not detections:
            # Publish empty
            empty_msg = BoundingBoxes()
            empty_msg.header = msg.header
            self.detections_pub.publish(empty_msg)
            return

        # NMS
        boxes = [d['box'] for d in detections]
        scores = [d['score'] for d in detections]
        indices = cv2.dnn.NMSBoxes(
            boxes, scores, self.nms_score_threshold, self.nms_threshold
        )

        # Prepare output messages
        bboxes_msg = BoundingBoxes()
        bboxes_msg.header = msg.header

        best_ball_center = None
        best_ball_score = -1.0
        best_ball_area = 0.0

        debug_image = bgr.copy() if self.publish_debug else None

        for idx in indices:
            det = detections[idx]
            x, y, w, h = det['box']
            class_id = det['class_id']
            score = det['score']

            # Create bounding box message
            bbox = BoundingBox()
            bbox.probability = float(score)
            bbox.xmin = int(x)
            bbox.ymin = int(y)
            bbox.xmax = int(x + w)
            bbox.ymax = int(y + h)
            bbox.class_id = self.CLASS_NAMES[class_id]
            bbox.id = int(class_id)

            cx = int(x + w / 2)
            cy = int(y + h / 2)
            bbox.xbase = cx
            bbox.ybase = int(y + h)
            bbox.obstacle_detected = False

            bboxes_msg.bounding_boxes.append(bbox)

            # Track best ball
            if class_id == 0:  # ball
                area = w * h
                if best_ball_center is None or area > best_ball_area or \
                   (abs(area - best_ball_area) < 1e-3 and score > best_ball_score):
                    best_ball_center = (cx, cy)
                    best_ball_score = score
                    best_ball_area = area

            # Debug visualization
            if self.publish_debug:
                color = self.CLASS_COLORS[class_id]
                cv2.rectangle(debug_image, (int(x), int(y)), (int(x+w), int(y+h)), color, 2)
                label = f'{self.CLASS_NAMES[class_id]} {score:.2f}'
                cv2.putText(debug_image, label, (int(x), max(0, int(y) - 5)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Publish detections
        self.detections_pub.publish(bboxes_msg)

        # Publish ball center
        if best_ball_center and width > 0 and height > 0:
            center_msg = PointStamped()
            center_msg.header = msg.header
            center_msg.point.x = max(0.0, min(1.0, best_ball_center[0] / width))
            center_msg.point.y = max(0.0, min(1.0, best_ball_center[1] / height))
            center_msg.point.z = best_ball_score
            self.ball_center_pub.publish(center_msg)

        # Publish debug image
        if self.publish_debug:
            debug_msg = self.cv2_to_ros(debug_image, msg.header)
            self.debug_pub.publish(debug_msg)

        # Performance logging
        end_time = time.perf_counter()
        full_ms = (end_time - start_time) * 1000

        self.frame_count += 1
        self.total_inference_ms += inference_ms
        self.total_full_ms += full_ms

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 >= 5.0:
            avg_inference_ms = self.total_inference_ms / self.frame_count
            avg_full_ms = self.total_full_ms / self.frame_count
            effective_fps = 1000.0 / avg_full_ms
            self.get_logger().info(
                f'Performance: inference={avg_inference_ms:.1f}ms, '
                f'total={avg_full_ms:.1f}ms, {effective_fps:.1f} FPS '
                f'(skip={self.frame_skip}, backend={self.dnn_backend}/{self.dnn_target})'
            )
            self.frame_count = 0
            self.total_inference_ms = 0.0
            self.total_full_ms = 0.0
            self.last_log_time = now

    def decode_detections(self, output, scale, image_width, image_height):
        """Decode YOLO output to detections."""
        detections = []

        # Handle different output shapes
        if len(output.shape) == 3:
            dim1, dim2 = output.shape[1], output.shape[2]
            if dim1 in [10, 11]:  # 4+6 or 5+6 classes
                output = output[0].T
            elif dim2 in [10, 11]:
                output = output[0]

        num_classes = len(self.CLASS_NAMES)

        for row in output:
            cx, cy, w, h = row[0:4]

            # Check if objectness exists
            has_objectness = len(row) >= (5 + num_classes)
            class_offset = 5 if has_objectness else 4
            objectness = row[4] if has_objectness else 1.0

            # Find best class
            class_scores = row[class_offset:class_offset + num_classes] * objectness
            best_class = int(np.argmax(class_scores))
            best_score = float(class_scores[best_class])

            if best_class < 0 or best_class >= num_classes:
                continue

            if best_score < self.class_thresholds[best_class]:
                continue

            # Convert to pixel coordinates
            left = int((cx - 0.5 * w) * scale)
            top = int((cy - 0.5 * h) * scale)
            width = int(w * scale)
            height = int(h * scale)

            # Clamp to image bounds
            left = max(0, min(left, image_width - 1))
            top = max(0, min(top, image_height - 1))
            width = max(1, min(width, image_width - left))
            height = max(1, min(height, image_height - top))

            detections.append({
                'box': [left, top, width, height],
                'class_id': best_class,
                'score': best_score
            })

        return detections

    def ros_to_cv2(self, msg):
        """Convert ROS Image message to OpenCV BGR image."""
        encoding = msg.encoding
        width, height = msg.width, msg.height

        if encoding == 'bgr8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            return img
        elif encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif encoding == 'rgba8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif encoding == 'bgra8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        elif encoding == 'mono8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            raise ValueError(f'Unsupported encoding: {encoding}')

    def cv2_to_ros(self, img, header):
        """Convert OpenCV BGR image to ROS Image message."""
        msg = Image()
        msg.header = header
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
