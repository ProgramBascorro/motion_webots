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

"""YOLO-based field (lapangan) detector for ROBOTIS OP3.

Unlike ``yolo_ball_detector.py`` — which collapses every target into a single
"ball" circle (``op3_ball_detector_msgs/CircleSetStamped``) and therefore throws
away the class label — this node runs a MULTI-CLASS YOLO model (e.g. field
lines, goal, penalty spot, corners) and publishes every detection WITH its
class label and score as a standard ``vision_msgs/Detection2DArray`` on
``detections``.

    detections[i].results[0].hypothesis.class_id : model class name (string)
    detections[i].results[0].hypothesis.score    : confidence [0..1]
    detections[i].bbox.center.position.x / .y     : box center in PIXELS
    detections[i].bbox.size_x / .size_y           : box size in PIXELS
    detections[i].id                              : class name (convenience)

An annotated debug image is published on ``image_out`` and the node honors the
``enable`` (std_msgs/Bool) topic just like the ball detector.

This runs as its OWN node/namespace (``field_detector_node``) so it does NOT
interfere with the ball_detector / ball_follower pipeline.

Default: detect ALL classes the model exposes. Set ``target_classes`` to a
subset of the model's class names to filter.
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
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from cv_bridge import CvBridge


class YoloFieldDetector(Node):
    """Detects field elements with a multi-class YOLO model."""

    def __init__(self):
        super().__init__('yolo_field_detector')

        # ----- parameters -------------------------------------------------
        self.model_path = str(self.declare_parameter('model_path', '').value)
        # Task for the model. OpenVINO/.onnx carry no task metadata ultralytics
        # detects reliably, so set it explicitly. Empty = let ultralytics infer.
        self.model_task = str(
            self.declare_parameter('model_task', 'detect').value).strip()
        self.use_compressed = bool(
            self.declare_parameter('use_compressed', True).value)
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.5).value)
        self.iou_threshold = float(
            self.declare_parameter('iou_threshold', 0.45).value)
        self.imgsz = int(self.declare_parameter('imgsz', 320).value)
        self.device = str(self.declare_parameter('device', 'cpu').value)
        self.half = bool(self.declare_parameter('half', False).value)
        # Ultralytics deprecated the predict-time 'half' arg: passing it at all
        # (even =False) warns on every call. Only forward an FP16 request when
        # explicitly enabled. (OpenVINO CPU precision is fixed at export time.)
        self._predict_precision = {'half': True} if self.half else {}
        self.max_detections = int(
            self.declare_parameter('max_detections', 30).value)
        # Subset of model class names to keep. EMPTY = keep every class (the
        # usual choice for a field detector, where you want all classes).
        # NOTE: an empty array ("target_classes: []") comes back as None because
        # ROS 2 cannot infer the element type of an empty list, so guard it.
        _target_classes = self.declare_parameter('target_classes', []).value
        self.target_classes = list(_target_classes) if _target_classes else []
        self.detection_rate = float(
            self.declare_parameter('detection_rate', 15.0).value)
        self.publish_image = bool(
            self.declare_parameter('publish_image', True).value)
        self.detection_frame_id = str(
            self.declare_parameter('detection_frame_id', 'detector').value)
        self.enabled = bool(
            self.declare_parameter('enable_at_start', True).value)

        if self.detection_rate <= 0.0:
            self.get_logger().warn('detection_rate <= 0, defaulting to 15 Hz')
            self.detection_rate = 15.0

        # ----- load the YOLO model ---------------------------------------
        self.model = self._load_model()

        # ----- ROS interface ---------------------------------------------
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_image = None        # (cv_bgr, header)

        self.detections_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
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
            'YOLO field detector ready '
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
            self.get_logger().fatal(
                'model_path is empty. Point it at a field OpenVINO model dir, '
                'e.g. model/v8n_lapangan_best_openvino_model')
            raise ValueError('model_path is required')
        if not os.path.isabs(model_path) and not os.path.exists(model_path):
            # A relative path like "model/v8n_lapangan_best_openvino_model" (an
            # OpenVINO directory) or "model/v8n_lapangan_best.pt" is resolved
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

        self.class_names = dict(model.names)
        self.target_class_ids = self._resolve_target_class_ids()
        self.get_logger().info(
            'Model classes: %s | keeping ids: %s'
            % (self.class_names,
               self.target_class_ids if self.target_class_ids else 'ALL'))

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

        Returns None to mean "keep every class" (the field-detector default).
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
                'keeping ALL classes.')
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
            # Publish an empty detection set (so consumers see "nothing") but
            # still stream the raw image for debugging.
            self._publish_detections([], header)
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

        speed = getattr(results[0], 'speed', None) if results else None
        if speed:
            pre = speed.get('preprocess', 0.0)
            inf = speed.get('inference', 0.0)
            post = speed.get('postprocess', 0.0)
            self.get_logger().info(
                'latency: pre=%.1f infer=%.1f post=%.1f ms (total=%.1f ms)'
                % (pre, inf, post, pre + inf + post),
                throttle_duration_sec=5.0)

        detections = self._extract_detections(results)
        self._publish_detections(detections, header)
        self._publish_annotated(results, cv_img, header)

    def _extract_detections(self, results):
        """Convert YOLO boxes to (cx, cy, w, h, class_name, conf) tuples (px)."""
        detections = []
        if not results:
            return detections
        result = results[0]
        boxes = getattr(result, 'boxes', None)
        if boxes is None or boxes.xyxy is None:
            return detections

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy() if boxes.conf is not None else None
        clses = boxes.cls.cpu().numpy() if boxes.cls is not None else None
        for idx, (x1, y1, x2, y2) in enumerate(xyxy):
            cx = float((x1 + x2) * 0.5)
            cy = float((y1 + y2) * 0.5)
            w = float(x2 - x1)
            h = float(y2 - y1)
            conf = float(confs[idx]) if confs is not None else 1.0
            cls_id = int(clses[idx]) if clses is not None else -1
            name = str(self.class_names.get(cls_id, cls_id))
            detections.append((cx, cy, w, h, name, conf))
        return detections

    def _publish_detections(self, detections, header):
        msg = Detection2DArray()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.detection_frame_id
        for cx, cy, w, h, name, conf in detections:
            det = Detection2D()
            det.header = msg.header
            det.id = name
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = conf
            det.results.append(hyp)
            msg.detections.append(det)
        self.detections_pub.publish(msg)

        if detections:
            names = {}
            for _, _, _, _, name, _ in detections:
                names[name] = names.get(name, 0) + 1
            summary = ', '.join('%s x%d' % (n, c) for n, c in names.items())
            self.get_logger().info(
                'field: %d detection(s): %s' % (len(detections), summary),
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info('no field detection',
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
    node = YoloFieldDetector()
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
