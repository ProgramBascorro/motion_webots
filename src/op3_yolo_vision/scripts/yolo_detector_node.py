#!/usr/bin/env python3
"""YOLO detector node for ROS2 — onnxruntime backend (OpenCV 4.5.4 incompatible with YOLOv8)."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from soccer_msgs.msg import BoundingBox, BoundingBoxes
import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import time

# onnxruntime — primary inference backend (cv2.dnn cannot load YOLOv8 attention ONNX on OpenCV 4.5.4)
try:
    import onnxruntime as ort
    ORT_AVAILABLE = True
except ImportError:
    ORT_AVAILABLE = False

# PIL — for letterbox resize (cv2 broken with numpy 2.x)
try:
    from PIL import Image as PILImage
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

# cv2 — only used for debug visualization; may be unavailable when numpy 2.x is active
try:
    import cv2
    CV2_AVAILABLE = True
except Exception:
    CV2_AVAILABLE = False


def _nms_numpy(boxes, scores, iou_threshold):
    """Pure-numpy NMS. boxes: [[x,y,w,h],...], returns surviving indices."""
    if not boxes:
        return []
    x1 = np.array([b[0] for b in boxes], dtype=np.float32)
    y1 = np.array([b[1] for b in boxes], dtype=np.float32)
    x2 = x1 + np.array([b[2] for b in boxes], dtype=np.float32)
    y2 = y1 + np.array([b[3] for b in boxes], dtype=np.float32)
    areas = (x2 - x1) * (y2 - y1)
    order = np.argsort(scores)[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]
    return keep


class YoloDetectorNode(Node):
    """YOLO-based object detector — onnxruntime inference, PIL/numpy preprocessing."""

    CLASS_NAMES = ['ball', 'goal post', 'robot', 'L-intersection', 'T-intersection', 'X-intersection']
    CLASS_COLORS = [
        (0, 255, 255),
        (0, 255, 0),
        (255, 0, 0),
        (255, 255, 0),
        (255, 0, 255),
        (0, 255, 255),
    ]

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/robotis_op3/camera/image_raw')
        self.declare_parameter('ball_center_topic', '/vision/yolo/ball_center')
        self.declare_parameter('model_path', 'models/yolo.onnx')
        self.declare_parameter('use_gpu', False)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('ball_confidence_threshold', 0.2)
        self.declare_parameter('goalpost_confidence_threshold', 0.2)
        self.declare_parameter('robot_confidence_threshold', 0.2)
        self.declare_parameter('intersection_confidence_threshold', 0.4)
        self.declare_parameter('nms_threshold', 0.5)
        self.declare_parameter('nms_score_threshold', 0.1)
        self.declare_parameter('publish_debug', False)
        self.declare_parameter('frame_skip', 1)

        self.image_topic = self.get_parameter('image_topic').value
        self.ball_center_topic = self.get_parameter('ball_center_topic').value
        self.model_path = self._resolve_model_path(self.get_parameter('model_path').value)
        self.use_gpu = self.get_parameter('use_gpu').value
        self.input_size = self.get_parameter('input_size').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.nms_score_threshold = self.get_parameter('nms_score_threshold').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.frame_skip = max(1, self.get_parameter('frame_skip').value)

        self.class_thresholds = [
            self.get_parameter('ball_confidence_threshold').value,
            self.get_parameter('goalpost_confidence_threshold').value,
            self.get_parameter('robot_confidence_threshold').value,
            self.get_parameter('intersection_confidence_threshold').value,
            self.get_parameter('intersection_confidence_threshold').value,
            self.get_parameter('intersection_confidence_threshold').value,
        ]

        self.frame_counter = 0
        self.frame_count = 0
        self.total_inference_ms = 0.0
        self.total_full_ms = 0.0
        self.last_log_time = self.get_clock().now()

        self._ort_session = None
        self._input_name = None
        self.model_loaded = self._load_model()

        self.debug_pub = self.create_publisher(Image, '/vision/yolo/debug', 10)
        self.detections_pub = self.create_publisher(BoundingBoxes, '/vision/yolo/detections', 10)
        if self.ball_center_topic:
            self.ball_center_pub = self.create_publisher(PointStamped, self.ball_center_topic, 10)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, 10)

        self.get_logger().info('YOLO detector initialized (onnxruntime backend)')
        self.get_logger().info(f'  Image topic: {self.image_topic}')
        self.get_logger().info(f'  Model: {self.model_path}')
        self.get_logger().info(f'  ORT available: {ORT_AVAILABLE}, PIL available: {PIL_AVAILABLE}')
        if not ORT_AVAILABLE:
            self.get_logger().error('onnxruntime not installed — run: pip3 install onnxruntime')
        if not PIL_AVAILABLE:
            self.get_logger().error('Pillow not installed — run: pip3 install Pillow')

    def _resolve_model_path(self, model_path):
        if not model_path:
            return model_path
        path = Path(model_path)
        if path.is_absolute():
            return str(path)
        pkg_share = get_package_share_directory('op3_yolo_vision')
        return str(Path(pkg_share) / model_path)

    def _load_model(self):
        if not ORT_AVAILABLE:
            self.get_logger().error('onnxruntime not available — cannot load model')
            return False
        if not PIL_AVAILABLE:
            self.get_logger().error('Pillow not available — cannot preprocess images')
            return False
        if not self.model_path or not Path(self.model_path).exists():
            self.get_logger().error(f'YOLO model not found: {self.model_path}')
            return False
        try:
            providers = ['CPUExecutionProvider']
            if self.use_gpu:
                try:
                    available = [p for p in ort.get_available_providers()]
                    if 'CUDAExecutionProvider' in available:
                        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
                        self.get_logger().info('Using CUDA execution provider')
                except Exception:
                    pass

            self._ort_session = ort.InferenceSession(self.model_path, providers=providers)
            self._input_name = self._ort_session.get_inputs()[0].name
            output_shape = self._ort_session.get_outputs()[0].shape
            self.get_logger().info(
                f'Model loaded via onnxruntime | output shape: {output_shape}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model with onnxruntime: {e}')
            return False

    def _preprocess(self, msg):
        """Convert ROS Image → NCHW float32 blob for onnxruntime. Returns (blob, scale, orig_w, orig_h)."""
        enc = msg.encoding
        h, w = msg.height, msg.width

        raw = np.frombuffer(msg.data, dtype=np.uint8)

        if enc == 'bgra8':
            img = raw.reshape(h, w, 4)
            rgb = img[:, :, [2, 1, 0]]  # BGRA → RGB (drop A)
        elif enc == 'bgr8':
            img = raw.reshape(h, w, 3)
            rgb = img[:, :, ::-1]
        elif enc == 'rgb8':
            rgb = raw.reshape(h, w, 3)
        elif enc == 'rgba8':
            img = raw.reshape(h, w, 4)
            rgb = img[:, :, :3]
        elif enc == 'mono8':
            gray = raw.reshape(h, w)
            rgb = np.stack([gray, gray, gray], axis=2)
        else:
            raise ValueError(f'Unsupported encoding: {enc}')

        # Letterbox: pad to square
        max_dim = max(w, h)
        if w != h:
            square = np.full((max_dim, max_dim, 3), 114, dtype=np.uint8)
            square[:h, :w] = rgb
            rgb = square

        scale = max_dim / self.input_size

        # Resize with PIL (avoids cv2 numpy 2.x issue)
        pil_img = PILImage.fromarray(rgb.astype(np.uint8))
        pil_resized = pil_img.resize((self.input_size, self.input_size), PILImage.BILINEAR)
        rgb_f = np.array(pil_resized, dtype=np.float32) / 255.0  # (H, W, 3) float32

        # HWC → NCHW
        blob = np.transpose(rgb_f, (2, 0, 1))[np.newaxis, :]  # (1, 3, H, W)
        return blob, scale, w, h

    def _image_callback(self, msg):
        if not self.model_loaded:
            return

        self.frame_counter += 1
        if self.frame_counter % self.frame_skip != 0:
            return

        start_time = time.perf_counter()

        try:
            blob, scale, img_w, img_h = self._preprocess(msg)
        except Exception as e:
            self.get_logger().warn(f'Preprocess failed: {e}')
            return

        try:
            outputs = self._ort_session.run(None, {self._input_name: blob})
        except Exception as e:
            self.get_logger().warn(f'Inference failed: {e}')
            return

        inference_ms = (time.perf_counter() - start_time) * 1000

        raw_output = outputs[0]  # (1, 10, 8400) or (1, 8400, 10)
        detections = self._decode(raw_output, scale, img_w, img_h)

        bboxes_msg = BoundingBoxes()
        bboxes_msg.header = msg.header

        if not detections:
            self.detections_pub.publish(bboxes_msg)
            return

        boxes = [d['box'] for d in detections]
        scores = [d['score'] for d in detections]

        # NMS — prefer cv2 if available, else pure numpy
        if CV2_AVAILABLE:
            try:
                indices = cv2.dnn.NMSBoxes(
                    boxes, scores, self.nms_score_threshold, self.nms_threshold)
                indices = [int(i) for i in indices]
            except Exception:
                indices = _nms_numpy(boxes, scores, self.nms_threshold)
        else:
            indices = _nms_numpy(boxes, scores, self.nms_threshold)

        best_ball_center = None
        best_ball_score = -1.0
        best_ball_area = 0.0

        for idx in indices:
            det = detections[idx]
            x, y, bw, bh = det['box']
            class_id = det['class_id']
            score = det['score']

            bbox = BoundingBox()
            bbox.probability = float(score)
            bbox.xmin = int(x)
            bbox.ymin = int(y)
            bbox.xmax = int(x + bw)
            bbox.ymax = int(y + bh)
            bbox.class_id = self.CLASS_NAMES[class_id]
            bbox.id = int(class_id)
            cx = int(x + bw / 2)
            bbox.xbase = cx
            bbox.ybase = int(y + bh)
            bbox.obstacle_detected = False
            bboxes_msg.bounding_boxes.append(bbox)

            if class_id == 0:
                area = bw * bh
                if best_ball_center is None or area > best_ball_area or \
                        (abs(area - best_ball_area) < 1e-3 and score > best_ball_score):
                    best_ball_center = (cx, int(y + bh / 2))
                    best_ball_score = score
                    best_ball_area = area

        self.detections_pub.publish(bboxes_msg)

        if best_ball_center and img_w > 0 and img_h > 0:
            pt_msg = PointStamped()
            pt_msg.header = msg.header
            pt_msg.point.x = max(0.0, min(1.0, best_ball_center[0] / img_w))
            pt_msg.point.y = max(0.0, min(1.0, best_ball_center[1] / img_h))
            pt_msg.point.z = float(best_ball_score)
            self.ball_center_pub.publish(pt_msg)

        full_ms = (time.perf_counter() - start_time) * 1000
        self.frame_count += 1
        self.total_inference_ms += inference_ms
        self.total_full_ms += full_ms

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 >= 5.0:
            avg_inf = self.total_inference_ms / self.frame_count
            avg_full = self.total_full_ms / self.frame_count
            self.get_logger().info(
                f'Perf: infer={avg_inf:.1f}ms total={avg_full:.1f}ms '
                f'{1000.0/avg_full:.1f}FPS skip={self.frame_skip}')
            self.frame_count = 0
            self.total_inference_ms = 0.0
            self.total_full_ms = 0.0
            self.last_log_time = now

    def _decode(self, output, scale, image_width, image_height):
        """Decode YOLOv8 ONNX output → list of detections."""
        detections = []

        if len(output.shape) == 3:
            d1, d2 = output.shape[1], output.shape[2]
            num_classes = len(self.CLASS_NAMES)
            if d1 in range(num_classes + 4, num_classes + 6):
                output = output[0].T   # (8400, 10)
            elif d2 in range(num_classes + 4, num_classes + 6):
                output = output[0]     # already (8400, 10)
            else:
                output = output[0].T

        for row in output:
            cx, cy, bw, bh = row[0:4]
            has_obj = len(row) >= (5 + len(self.CLASS_NAMES))
            cls_off = 5 if has_obj else 4
            obj = row[4] if has_obj else 1.0
            cls_scores = row[cls_off:cls_off + len(self.CLASS_NAMES)] * obj
            best_cls = int(np.argmax(cls_scores))
            best_score = float(cls_scores[best_cls])

            if best_cls < 0 or best_cls >= len(self.CLASS_NAMES):
                continue
            if best_score < self.class_thresholds[best_cls]:
                continue

            left = int((cx - 0.5 * bw) * scale)
            top = int((cy - 0.5 * bh) * scale)
            width = int(bw * scale)
            height = int(bh * scale)
            left = max(0, min(left, image_width - 1))
            top = max(0, min(top, image_height - 1))
            width = max(1, min(width, image_width - left))
            height = max(1, min(height, image_height - top))

            detections.append({'box': [left, top, width, height],
                                'class_id': best_cls, 'score': best_score})
        return detections


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
