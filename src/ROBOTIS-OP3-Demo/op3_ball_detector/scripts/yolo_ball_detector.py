#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 ROBOTIS / Bascorro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""YOLO-based ball detector for ROBOTIS OP3.

This node is a drop-in replacement for the classic Hough-circle
``ball_detector_node`` (C++). It runs an Ultralytics YOLO model on the
camera stream and publishes the detected ball(s) as
``op3_ball_detector_msgs/CircleSetStamped`` on ``circle_set`` using the very
same convention as the original detector so the existing op3_demo
ball_tracker / ball_follower pipeline keeps working unchanged:

    circles[i].x : normalized x in [-1, 1]   (image left -1 ... right +1)
    circles[i].y : normalized y in [-1, 1]   (image top  -1 ... bottom +1)
    circles[i].z : ball radius in pixels

An annotated debug image is published on ``image_out`` and the node honors
the ``enable`` (std_msgs/Bool) topic exactly like the C++ detector.

Default model: <op3_ball_detector share>/model/best.pt  (class 0 = "Bola-baru")

Run under the ``ball_detector_node`` namespace so it publishes on
``/ball_detector_node/circle_set`` (see yolo_ball_detector.launch.py).
"""

import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from op3_ball_detector_msgs.msg import CircleSetStamped

from cv_bridge import CvBridge


class YoloBallDetector(Node):
    """Detects the ball with a YOLO model and republishes it as circles."""

    def __init__(self):
        super().__init__('yolo_ball_detector')

        # ----- parameters -------------------------------------------------
        self.model_path = str(self.declare_parameter('model_path', '').value)
        # Task for the model. .onnx files carry no task metadata, so ultralytics
        # otherwise warns and guesses; set it explicitly. Empty = let ultralytics
        # infer (only works for .pt).
        self.model_task = str(
            self.declare_parameter('model_task', 'detect').value).strip()
        self.use_compressed = bool(
            self.declare_parameter('use_compressed', True).value)
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.5).value)
        self.iou_threshold = float(
            self.declare_parameter('iou_threshold', 0.45).value)
        self.imgsz = int(self.declare_parameter('imgsz', 640).value)
        self.device = str(self.declare_parameter('device', 'cpu').value)
        self.half = bool(self.declare_parameter('half', False).value)
        # Ultralytics deprecated the predict-time 'half' argument: passing it at
        # all (even =False) prints a "'half' is deprecated, use 'quantize'"
        # warning on every inference call. Forward an FP16 request only when half
        # is explicitly enabled; otherwise omit the kwarg so the default path
        # stays quiet. (OpenVINO CPU precision is fixed at export time anyway.)
        self._predict_precision = {'half': True} if self.half else {}
        self.max_detections = int(
            self.declare_parameter('max_detections', 10).value)
        # Class names (from the model) that should be treated as "the ball".
        self.target_classes = list(
            self.declare_parameter('target_classes', ['Bola-baru']).value)
        self.detection_rate = float(
            self.declare_parameter('detection_rate', 30.0).value)
        self.publish_image = bool(
            self.declare_parameter('publish_image', True).value)
        self.detection_frame_id = str(
            self.declare_parameter('detection_frame_id', 'detector').value)
        self.enabled = bool(
            self.declare_parameter('enable_at_start', True).value)

        if self.detection_rate <= 0.0:
            self.get_logger().warn('detection_rate <= 0, defaulting to 30 Hz')
            self.detection_rate = 30.0

        # ----- load the YOLO model ---------------------------------------
        self.model = self._load_model()

        # ----- ROS interface ---------------------------------------------
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_image = None        # (cv_bgr, header)
        self._processed_stamp = None     # last header processed (dedupe)

        self.circles_pub = self.create_publisher(
            CircleSetStamped, 'circle_set', 10)
        if self.publish_image:
            self.image_pub = self.create_publisher(Image, 'image_out', 1)
        else:
            self.image_pub = None

        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage, 'image_in',
                self._compressed_image_callback, qos_profile_sensor_data)
        else:
            self.image_sub = self.create_subscription(
                Image, 'image_in',
                self._image_callback, qos_profile_sensor_data)

        self.enable_sub = self.create_subscription(
            Bool, 'enable', self._enable_callback, 1)

        # Run inference on the freshest frame at a fixed rate. Decoupling the
        # camera rate from inference avoids queue build-up when YOLO is slow.
        self.timer = self.create_timer(
            1.0 / self.detection_rate, self._process_latest)

        self.get_logger().info(
            'YOLO ball detector ready '
            f'(compressed={self.use_compressed}, conf={self.confidence_threshold}, '
            f'device={self.device}, rate={self.detection_rate:.1f}Hz, '
            f'enabled={self.enabled})')

    # ---------------------------------------------------------------------
    # setup helpers
    # ---------------------------------------------------------------------
    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().fatal(
                'Failed to import ultralytics. Install it with '
                '"pip3 install ultralytics". Error: %s' % exc)
            raise

        share_dir = get_package_share_directory('op3_ball_detector')
        model_path = self.model_path.strip()
        if not model_path:
            # No model configured: use the package default.
            model_path = os.path.join(share_dir, 'model', 'best.pt')
        elif not os.path.isabs(model_path) and not os.path.exists(model_path):
            # A relative path like "model/v8s.pt" (a .pt/.onnx file) or
            # "model/v8n_openvino_model" (an OpenVINO directory) is resolved
            # against the package share dir (where install(DIRECTORY ... model)
            # puts it), so editing model_path in the YAML "just works".
            model_path = os.path.join(share_dir, model_path)

        # An OpenVINO model is a directory (.xml/.bin); .pt/.onnx are files.
        if not os.path.exists(model_path):
            self.get_logger().fatal('Model path not found: %s' % model_path)
            raise FileNotFoundError(model_path)

        self.get_logger().info('Loading YOLO model: %s' % model_path)
        model = (YOLO(model_path, task=self.model_task)
                 if self.model_task else YOLO(model_path))

        # Resolve target class names -> class indices for the model.
        self.class_names = dict(model.names)
        self.target_class_ids = self._resolve_target_class_ids()
        self.get_logger().info(
            'Model classes: %s | targeting ids: %s'
            % (self.class_names, self.target_class_ids))

        # Warm-up so the first real frame is not penalised by lazy init.
        try:
            import numpy as np
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            model.predict(dummy, imgsz=self.imgsz, device=self.device,
                          verbose=False, **self._predict_precision)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('Model warm-up skipped: %s' % exc)

        return model

    def _resolve_target_class_ids(self):
        """Map configured target class names to model class indices.

        Returns None to mean "detect every class" when no target name matches.
        """
        if not self.target_classes:
            return None
        name_to_id = {str(name).lower(): idx
                      for idx, name in self.class_names.items()}
        ids = []
        for wanted in self.target_classes:
            key = str(wanted).lower()
            if key in name_to_id:
                ids.append(name_to_id[key])
            else:
                self.get_logger().warn(
                    'target class "%s" not in model; ignoring' % wanted)
        if not ids:
            self.get_logger().warn(
                'No configured target class matched the model classes; '
                'falling back to detecting ALL classes as ball.')
            return None
        return ids

    # ---------------------------------------------------------------------
    # subscriptions
    # ---------------------------------------------------------------------
    def _enable_callback(self, msg: Bool):
        if msg.data != self.enabled:
            self.get_logger().info(
                'Detection %s' % ('ENABLED' if msg.data else 'DISABLED'))
        self.enabled = msg.data

    def _compressed_image_callback(self, msg: CompressedImage):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('compressed cv_bridge failed: %s' % exc)
            return
        with self._frame_lock:
            self._latest_image = (cv_img, msg.header)

    def _image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge failed: %s' % exc)
            return
        with self._frame_lock:
            self._latest_image = (cv_img, msg.header)

    # ---------------------------------------------------------------------
    # main detection loop
    # ---------------------------------------------------------------------
    def _process_latest(self):
        with self._frame_lock:
            data = self._latest_image
            self._latest_image = None
        if data is None:
            return
        cv_img, header = data

        if not self.enabled:
            # Mirror the C++ detector: do not publish circles while disabled,
            # but still stream the (raw) image for debugging.
            self._publish_image(cv_img, header)
            return

        try:
            results = self.model.predict(
                cv_img,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
                max_det=self.max_detections,
                classes=self.target_class_ids,
                verbose=False,
                **self._predict_precision)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('YOLO inference failed: %s' % exc,
                                    throttle_duration_sec=2.0)
            return

        # Periodic latency readout (ultralytics reports per-stage ms in
        # results[].speed). Throttled so it never floods the console; use it to
        # measure the effect of imgsz / INT8 / thread changes.
        speed = getattr(results[0], 'speed', None) if results else None
        if speed:
            pre = speed.get('preprocess', 0.0)
            inf = speed.get('inference', 0.0)
            post = speed.get('postprocess', 0.0)
            self.get_logger().info(
                'latency: pre=%.1f infer=%.1f post=%.1f ms (total=%.1f ms)'
                % (pre, inf, post, pre + inf + post),
                throttle_duration_sec=5.0)

        height, width = cv_img.shape[:2]
        circles = self._extract_circles(results, width, height)
        self._publish_circles(circles, header)
        self._publish_annotated(results, cv_img, header)

    def _extract_circles(self, results, width, height):
        """Convert YOLO boxes to (norm_x, norm_y, radius_px, conf) tuples."""
        circles = []
        if not results:
            return circles
        result = results[0]
        boxes = getattr(result, 'boxes', None)
        if boxes is None or boxes.xyxy is None:
            return circles

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy() if boxes.conf is not None else None
        for idx, (x1, y1, x2, y2) in enumerate(xyxy):
            cx = (x1 + x2) * 0.5
            cy = (y1 + y2) * 0.5
            radius = ((x2 - x1) + (y2 - y1)) * 0.25  # mean half-side, in px
            norm_x = (cx / width) * 2.0 - 1.0
            norm_y = (cy / height) * 2.0 - 1.0
            conf = float(confs[idx]) if confs is not None else 1.0
            circles.append((norm_x, norm_y, float(radius), conf))

        # Largest ball first; downstream picks the circle with the biggest z.
        circles.sort(key=lambda c: c[2], reverse=True)
        return circles

    def _publish_circles(self, circles, header):
        msg = CircleSetStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.detection_frame_id
        msg.circles = [Point(x=c[0], y=c[1], z=c[2]) for c in circles]
        self.circles_pub.publish(msg)

        if circles:
            best = circles[0]
            self.get_logger().info(
                'ball: center=(%.2f, %.2f) radius=%.1fpx conf=%.2f (%d total)'
                % (best[0], best[1], best[2], best[3], len(circles)),
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info('no ball detected',
                                   throttle_duration_sec=3.0)

    def _publish_annotated(self, results, cv_img, header):
        # Skip the costly plot()/encode/publish entirely when nobody subscribes
        # to image_out, so the debug overlay is free during real operation yet
        # still available on demand (open it in rqt/foxglove to resume).
        if self.image_pub is None or self.image_pub.get_subscription_count() == 0:
            return
        try:
            annotated = results[0].plot() if results else cv_img
        except Exception:  # noqa: BLE001
            annotated = cv_img
        self._publish_image(annotated, header)

    def _publish_image(self, cv_img, header):
        if self.image_pub is None or self.image_pub.get_subscription_count() == 0:
            return
        try:
            out = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
            out.header.stamp = header.stamp
            out.header.frame_id = self.detection_frame_id
            self.image_pub.publish(out)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('failed to publish image_out: %s' % exc,
                                   throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBallDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
