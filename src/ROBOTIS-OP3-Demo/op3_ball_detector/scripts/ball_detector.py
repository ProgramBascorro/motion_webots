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

"""Ball detector running the weights in ``model/ball_model``.

This is a standalone program built around the BALL. The weights may carry
more classes than the ball (the robot_torso model: ball, goalpost, robot,
plus l/t/x): ``target_classes`` picks which ones are detected at all, and
``ball_classes`` picks which of those count as "the ball". Only ball boxes go
to circle_set / ball_center; every targeted class (ball included) is
published on ``detections``. Field landmarks are handled by a separate
program, ``landmark_detector.py``, so the two can be started, stopped and
tuned independently.

It publishes the ball with the same convention the classic Hough-circle
``ball_detector_node`` (C++) uses, so the existing op3_demo ball_tracker /
ball_follower pipeline keeps working unchanged:

    circles[i].x : normalized x in [-1, 1]   (image left -1 ... right +1)
    circles[i].y : normalized y in [-1, 1]   (image top  -1 ... bottom +1)
    circles[i].z : ball radius in pixels

Topics (relative, so a namespace moves them all together):

    image_in    (sensor_msgs/CompressedImage or Image)  camera in
    enable      (std_msgs/Bool)                         gate detection
    circle_set  (op3_ball_detector_msgs/CircleSetStamped)  ball_classes only
    detections  (vision_msgs/Detection2DArray)  every target class, px units
    fps         (std_msgs/Float32)                      pipeline FPS, live
    image_out   (sensor_msgs/Image)                     annotated debug view
    /vision/yolo/ball_center (geometry_msgs/PointStamped, absolute)
                head_tracking_node feed, [0,1] image coords, only while seen

Weights are picked automatically from ``model/ball_model`` to match the
runtimes actually installed here: the OpenVINO IR directory when the openvino
package is present (the robot container), the .onnx when onnxruntime is
present, the .pt otherwise. Set the ``model_path`` parameter to override.

FPS is reported continuously: every ``fps_log_period`` seconds while running,
and once more as a summary when the node stops.

Run:
    ros2 launch op3_ball_detector ball_detector.launch.py
    ros2 run op3_ball_detector ball_detector.py --ros-args -p device:=cpu
"""

import array
import glob
import importlib.util
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy)
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, PointStamped
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)
from op3_ball_detector_msgs.msg import CircleSetStamped

import cv2
from cv_bridge import CvBridge


def _module_available(name):
    """True when `name` can be imported without actually importing it."""
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


class FpsMeter:
    """Frame/inference bookkeeping behind the FPS log lines.

    Two different rates matter and both get reported, because confusing them
    hides problems:

      * pipeline FPS - frames this node actually pushed through per second.
        Capped by ``detection_rate`` and by how fast the camera publishes.
      * model FPS    - 1 / inference time, i.e. what the weights could do if
        nothing else got in the way.
    """

    def __init__(self, window_sec):
        self.window_sec = max(float(window_sec), 0.1)
        self.first_frame_time = None
        self.last_frame_time = None
        self.total_frames = 0
        self.total_infer_sec = 0.0
        self.total_inputs = 0
        self.window_start = time.monotonic()
        self.window_frames = 0
        self.window_infer_sec = 0.0
        self.window_inputs = 0
        self.min_fps = None
        self.max_fps = None

    def count_input(self):
        """One camera frame arrived (whether or not it gets processed)."""
        self.total_inputs += 1
        self.window_inputs += 1

    def count_frame(self, infer_sec):
        """One frame went through the model, taking `infer_sec` seconds."""
        now = time.monotonic()
        if self.first_frame_time is None:
            self.first_frame_time = now
        self.last_frame_time = now
        self.total_frames += 1
        self.total_infer_sec += infer_sec
        self.window_frames += 1
        self.window_infer_sec += infer_sec

    def due(self, now):
        return (now - self.window_start) >= self.window_sec

    def take_window(self, now):
        """Close the running window and return (fps, infer_ms, cam_fps)."""
        elapsed = max(now - self.window_start, 1e-9)
        fps = self.window_frames / elapsed
        cam_fps = self.window_inputs / elapsed
        infer_ms = (self.window_infer_sec / self.window_frames * 1000.0
                    if self.window_frames else 0.0)
        # Windows without a single frame are camera problems, not slow
        # inference - keep them out of the min/max so the summary stays honest.
        if self.window_frames:
            self.min_fps = fps if self.min_fps is None else min(self.min_fps, fps)
            self.max_fps = fps if self.max_fps is None else max(self.max_fps, fps)
        self.window_start = now
        self.window_frames = 0
        self.window_infer_sec = 0.0
        self.window_inputs = 0
        return fps, infer_ms, cam_fps

    def summary_lines(self, title):
        """Multi-line end-of-run report."""
        if not self.total_frames:
            return ['%s: no frame was ever processed '
                    '(camera never published on image_in?)' % title]
        # Measure between the first and last frame, not up to "now": time
        # spent idle waiting for a camera that was not publishing is not a
        # frame rate, and averaging it in would put the average below the
        # per-window minimum.
        elapsed = self.last_frame_time - self.first_frame_time
        avg_fps = ('%.1f' % (self.total_frames / elapsed)
                   if elapsed > 0 else 'n/a')
        infer_ms = self.total_infer_sec / self.total_frames * 1000.0
        model_fps = 1000.0 / infer_ms if infer_ms > 0 else 0.0
        return [
            '===== %s: FPS summary =====' % title,
            'frames processed  : %d in %.1f s of camera feed'
            % (self.total_frames, elapsed),
            'camera frames in  : %d' % self.total_inputs,
            'pipeline FPS      : %s avg (min %s, max %s)'
            % (avg_fps,
               '%.1f' % self.min_fps if self.min_fps is not None else 'n/a',
               '%.1f' % self.max_fps if self.max_fps is not None else 'n/a'),
            'inference         : %.1f ms/frame  ->  %.1f FPS model-only'
            % (infer_ms, model_fps),
        ]


class BallDetector(Node):
    """Detects the ball with the model/ball_model weights."""

    # Where this program's weights live, relative to the package root.
    MODEL_DIR = os.path.join('model', 'ball_model')

    def __init__(self):
        super().__init__('ball_detector')

        # ----- parameters -------------------------------------------------
        # Empty model_path = auto-pick the best weights in MODEL_DIR for the
        # runtimes installed here (see _autopick_weights).
        self.model_path = str(self.declare_parameter('model_path', '').value).strip()
        # .onnx / OpenVINO carry no task metadata ultralytics can rely on, so
        # state it. Empty = let ultralytics infer (only reliable for .pt).
        self.model_task = str(
            self.declare_parameter('model_task', 'detect').value).strip()
        self.use_compressed = bool(
            self.declare_parameter('use_compressed', True).value)
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.5).value)
        self.iou_threshold = float(
            self.declare_parameter('iou_threshold', 0.45).value)
        # Must match the imgsz the weights were exported with (640 here).
        self.imgsz = int(self.declare_parameter('imgsz', 640).value)
        # Rectangular exports (e.g. 384x640, which fits a 16:9 camera with
        # almost no letterbox padding) cannot be described by a single number:
        # ultralytics reads an int as a SQUARE side. imgsz_hw spells the shape
        # out as [height, width]. [0, 0] = unused, keep the square imgsz above.
        # The value must match what the weights were exported with -- the
        # anchor grid is baked in at export time and cannot be reshaped later.
        hw = list(self.declare_parameter('imgsz_hw', [0, 0]).value or [])
        if len(hw) == 2 and int(hw[0]) > 0 and int(hw[1]) > 0:
            self.imgsz_arg = [int(hw[0]), int(hw[1])]
            self.imgsz_desc = '%dx%d' % (hw[0], hw[1])
        else:
            self.imgsz_arg = self.imgsz
            self.imgsz_desc = str(self.imgsz)
        self.device = str(self.declare_parameter('device', 'cpu').value)
        self.half = bool(self.declare_parameter('half', False).value)
        self.max_detections = int(
            self.declare_parameter('max_detections', 10).value)
        # Model class names to detect at all; everything else is dropped
        # inside predict(). Empty = keep every class of the model.
        self.target_classes = list(
            self.declare_parameter('target_classes', ['ball']).value)
        # Which of the detected classes are "the ball" -- only these reach
        # circle_set / ball_center, so a goalpost or robot box can never be
        # chased as a ball. Empty = every detected class is a ball (the old
        # single-class behaviour).
        self.ball_classes = list(
            self.declare_parameter('ball_classes', ['ball']).value)
        self.detection_rate = float(
            self.declare_parameter('detection_rate', 15.0).value)
        self.publish_image = bool(
            self.declare_parameter('publish_image', True).value)
        # JPEG quality of image_out/compressed. The raw image_out is ~675 KB
        # per 640x360 frame; at q70 the same annotated frame is ~33 KB, which
        # is what lets rqt_image_view keep up at 30 FPS.
        self.jpeg_quality = int(
            self.declare_parameter('image_out_jpeg_quality', 70).value)
        self.detection_frame_id = str(
            self.declare_parameter('detection_frame_id', 'detector').value)
        self.enabled = bool(
            self.declare_parameter('enable_at_start', True).value)
        # Head-tracking feed for op3_ball_localization/head_tracking_node.
        # Absolute topic so it stays global whatever namespace we run under.
        # Empty string disables the publisher.
        self.ball_center_topic = str(
            self.declare_parameter(
                'ball_center_topic', '/vision/yolo/ball_center').value).strip()
        # How often the FPS line is logged, in seconds.
        self.fps_log_period = float(
            self.declare_parameter('fps_log_period', 5.0).value)
        # Inferensi normal di CHRONUS ~20-22 ms. Di atas ambang ini CPU sedang
        # dibatasi (daya/baterai -- uji 2026-11-07 22:37: 68-83 ms lalu PC mati)
        # atau direbut proses lain (build): peringatkan, jangan diam-diam lambat.
        # 0 = mati.
        self.slow_infer_warn_ms = float(
            self.declare_parameter('slow_infer_warn_ms', 40.0).value)

        if self.detection_rate <= 0.0:
            self.get_logger().warn('detection_rate <= 0, defaulting to 15 Hz')
            self.detection_rate = 15.0

        self.fps = FpsMeter(self.fps_log_period)
        self._summary_logged = False

        # ----- model ------------------------------------------------------
        self.model = self._load_model()

        # ----- ROS interface ----------------------------------------------
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_image = None        # (cv_bgr, header)

        self.circles_pub = self.create_publisher(
            CircleSetStamped, 'circle_set', 10)
        # Every target class (ball, goalpost, robot, ...), same message and
        # layout as landmark_detector.py's detections topic.
        self.detections_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        # The same number the FPS log line carries, for rqt_plot and friends.
        self.fps_pub = self.create_publisher(Float32, 'fps', 1)
        self.ball_center_pub = (
            self.create_publisher(PointStamped, self.ball_center_topic, 10)
            if self.ball_center_topic else None)
        self.image_pub = (self.create_publisher(Image, 'image_out', 1)
                          if self.publish_image else None)
        # Same annotated view, JPEG-compressed. image_transport naming, so
        # rqt_image_view lists it as .../image_out/compressed.
        self.image_jpeg_pub = (
            self.create_publisher(CompressedImage, 'image_out/compressed', 1)
            if self.publish_image else None)

        # Depth 1, not qos_profile_sensor_data (depth 5): the executor takes
        # ONE message per wait cycle, so with a 5-deep queue and inference
        # about as slow as the camera, _latest_image could be a frame that sat
        # in the queue for up to ~4 x 33 ms. Depth 1 keeps only the newest
        # frame (2026-11-12, goalkeeper save response). Best effort, like
        # sensor_data, so it still matches a reliable or best-effort camera.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage, 'image_in',
                self._compressed_image_callback, image_qos)
        else:
            self.image_sub = self.create_subscription(
                Image, 'image_in',
                self._image_callback, image_qos)

        self.enable_sub = self.create_subscription(
            Bool, 'enable', self._enable_callback, 1)

        # Inference runs on the freshest frame at a fixed rate. Decoupling it
        # from the camera rate stops a queue building up when YOLO is slower
        # than the camera.
        self.timer = self.create_timer(
            1.0 / self.detection_rate, self._process_latest)

        self.get_logger().info(
            'ball detector ready (compressed=%s, conf=%.2f, imgsz=%s, '
            'device=%s, rate=%.1fHz, enabled=%s, fps log every %.1fs)'
            % (self.use_compressed, self.confidence_threshold, self.imgsz_desc,
               self.device, self.detection_rate, self.enabled,
               self.fps_log_period))

    # ---------------------------------------------------------------------
    # model loading
    # ---------------------------------------------------------------------
    def _package_roots(self):
        """Directories that may contain this package's `model/` tree.

        The installed share dir comes first, then the source checkout this
        file lives in, so the program also runs straight out of src/ without
        a colcon build.
        """
        roots = []
        try:
            from ament_index_python.packages import get_package_share_directory
            roots.append(get_package_share_directory('op3_ball_detector'))
        except Exception:  # noqa: BLE001 - package simply not installed yet
            pass
        roots.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        return roots

    def _resolve_model_dir(self):
        for root in self._package_roots():
            candidate = os.path.join(root, self.MODEL_DIR)
            if os.path.isdir(candidate):
                return candidate
        raise FileNotFoundError(
            '%s not found under any of: %s'
            % (self.MODEL_DIR, self._package_roots()))

    def _autopick_weights(self, model_dir):
        """Best weights in `model_dir` for the runtimes present on this box.

        Preference order is fastest-first among what can actually run:
        OpenVINO IR (Intel CPU runtime, what the robot container has), then
        ONNX (needs onnxruntime), then the .pt (torch, always available but
        the slowest of the three).
        """
        candidates = []
        if _module_available('openvino'):
            candidates += sorted(glob.glob(
                os.path.join(model_dir, '*_openvino_model')))
        if _module_available('onnxruntime'):
            candidates += sorted(glob.glob(os.path.join(model_dir, '*.onnx')))
        candidates += sorted(glob.glob(os.path.join(model_dir, '*.pt')))
        if not candidates:
            raise FileNotFoundError(
                'no runnable weights in %s (looked for an *_openvino_model '
                'dir, *.onnx, *.pt against the installed runtimes)' % model_dir)
        return candidates[0]

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().fatal(
                'Failed to import ultralytics: %s' % exc)
            raise

        if self.model_path:
            path = self.model_path
            if not os.path.isabs(path) and not os.path.exists(path):
                # Relative paths resolve against the package (share dir first).
                for root in self._package_roots():
                    candidate = os.path.join(root, path)
                    if os.path.exists(candidate):
                        path = candidate
                        break
            if not os.path.exists(path):
                self.get_logger().fatal('model_path not found: %s' % path)
                raise FileNotFoundError(path)
        else:
            path = self._autopick_weights(self._resolve_model_dir())

        backend = ('OpenVINO' if path.endswith('_openvino_model')
                   else 'ONNX' if path.endswith('.onnx') else 'PyTorch')
        self.get_logger().info('loading detector weights (%s): %s' % (backend, path))
        model = (YOLO(path, task=self.model_task)
                 if self.model_task else YOLO(path))

        self.class_names = dict(model.names)
        self.target_class_ids = self._resolve_class_ids(
            self.target_classes, 'target')
        self.ball_class_ids = self._resolve_class_ids(
            self.ball_classes, 'ball')
        self.get_logger().info(
            'model classes: %s | targeting ids: %s | ball ids: %s'
            % (self.class_names, self.target_class_ids, self.ball_class_ids))

        self._warmup(model)
        return model

    def _warmup(self, model):
        """Run a few dummy frames so the first real one is not penalised.

        The timing is also the first FPS number of the run: it says what the
        weights can do here before the camera is even involved.
        """
        try:
            import numpy as np
            if isinstance(self.imgsz_arg, list):
                dh, dw = self.imgsz_arg
            else:
                dh = dw = self.imgsz
            dummy = np.zeros((dh, dw, 3), dtype=np.uint8)
            model.predict(dummy, imgsz=self.imgsz_arg, device=self.device,
                          half=self.half, verbose=False)
            start = time.monotonic()
            runs = 3
            for _ in range(runs):
                model.predict(dummy, imgsz=self.imgsz_arg, device=self.device,
                              half=self.half, verbose=False)
            per_frame = (time.monotonic() - start) / runs
            self.get_logger().info(
                'warm-up: %.1f ms/frame -> %.1f FPS model-only (pipeline is '
                'capped at detection_rate=%.1f FPS)'
                % (per_frame * 1000.0, 1.0 / max(per_frame, 1e-9),
                   self.detection_rate))
            self._warn_if_slow(per_frame * 1000.0, 'warm-up')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('warm-up skipped: %s' % exc)

    def _resolve_class_ids(self, wanted_names, kind):
        """Map configured class names to model class indices.

        None means "every class", which is also the fallback when nothing
        matches - better a noisy detector than a silent one.
        """
        if not wanted_names:
            return None
        name_to_id = {str(name).lower(): idx
                      for idx, name in self.class_names.items()}
        ids = []
        for wanted in wanted_names:
            key = str(wanted).lower()
            if key in name_to_id:
                ids.append(name_to_id[key])
            else:
                self.get_logger().warn(
                    '%s class "%s" not in model; ignoring' % (kind, wanted))
        if not ids:
            self.get_logger().warn(
                'no configured %s class matched the model; falling back to '
                'ALL classes.' % kind)
            return None
        return ids

    # ---------------------------------------------------------------------
    # subscriptions
    # ---------------------------------------------------------------------
    def _enable_callback(self, msg: Bool):
        if msg.data != self.enabled:
            self.get_logger().info(
                'detection %s' % ('ENABLED' if msg.data else 'DISABLED'))
        self.enabled = msg.data

    # The callbacks only keep the newest message; it is decoded in
    # _process_latest. When inference falls behind the camera, frames get
    # replaced before they are used, and decoding them would be wasted work.
    def _compressed_image_callback(self, msg: CompressedImage):
        self.fps.count_input()
        with self._frame_lock:
            self._latest_image = msg

    def _image_callback(self, msg: Image):
        self.fps.count_input()
        with self._frame_lock:
            self._latest_image = msg

    def _decode(self, msg):
        try:
            if isinstance(msg, CompressedImage):
                return self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            return self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge failed: %s' % exc,
                                   throttle_duration_sec=2.0)
            return None

    # ---------------------------------------------------------------------
    # detection loop
    # ---------------------------------------------------------------------
    def _process_latest(self):
        with self._frame_lock:
            msg = self._latest_image
            self._latest_image = None

        detected = 0
        cv_img = self._decode(msg) if msg is not None else None
        if cv_img is not None:
            header = msg.header
            if self.enabled:
                detected = self._detect_and_publish(cv_img, header)
            else:
                # Mirror the C++ detector: no circles while disabled, but keep
                # streaming the raw image so the debug view does not freeze.
                self._publish_image(cv_img, header)

        self._log_fps(detected)

    def _detect_and_publish(self, cv_img, header):
        try:
            start = time.monotonic()
            results = self.model.predict(
                cv_img,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz_arg,
                device=self.device,
                half=self.half,
                max_det=self.max_detections,
                classes=self.target_class_ids,
                verbose=False)
            self.fps.count_frame(time.monotonic() - start)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('inference failed: %s' % exc,
                                    throttle_duration_sec=2.0)
            return 0

        height, width = cv_img.shape[:2]
        circles = self._extract_circles(results, width, height)
        self._publish_circles(circles, header)
        self._publish_detections(results, header)
        self._publish_annotated(results, cv_img, header)
        return len(circles)

    def _extract_circles(self, results, width, height):
        """YOLO boxes -> (norm_x, norm_y, radius_px, conf) tuples."""
        circles = []
        if not results:
            return circles
        boxes = getattr(results[0], 'boxes', None)
        if boxes is None or boxes.xyxy is None:
            return circles

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy() if boxes.conf is not None else None
        classes = boxes.cls.cpu().numpy() if boxes.cls is not None else None
        for idx, (x1, y1, x2, y2) in enumerate(xyxy):
            # Goalposts, robots, ... are NOT balls: keep them out of
            # circle_set, or the soccer demo would walk at them.
            if (self.ball_class_ids is not None and classes is not None
                    and int(classes[idx]) not in self.ball_class_ids):
                continue
            cx = (x1 + x2) * 0.5
            cy = (y1 + y2) * 0.5
            radius = ((x2 - x1) + (y2 - y1)) * 0.25  # mean half-side, px
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

        if circles and self.ball_center_pub is not None:
            best = circles[0]
            # circles carry x,y in [-1, 1]; head_tracking_node wants [0, 1].
            center = PointStamped()
            center.header.stamp = header.stamp
            center.header.frame_id = self.detection_frame_id
            center.point.x = (best[0] + 1.0) / 2.0
            center.point.y = (best[1] + 1.0) / 2.0
            center.point.z = best[3]   # confidence
            self.ball_center_pub.publish(center)

    def _publish_detections(self, results, header):
        """Every target class as vision_msgs/Detection2DArray, px units.

        Same layout as landmark_detector.py: id and class_id carry the class
        NAME, most confident first.
        """
        msg = Detection2DArray()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.detection_frame_id
        boxes = getattr(results[0], 'boxes', None) if results else None
        if boxes is not None and boxes.xyxy is not None:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = (boxes.conf.cpu().numpy()
                     if boxes.conf is not None else None)
            classes = (boxes.cls.cpu().numpy()
                       if boxes.cls is not None else None)
            order = (sorted(range(len(xyxy)), key=lambda i: -float(confs[i]))
                     if confs is not None else range(len(xyxy)))
            for idx in order:
                x1, y1, x2, y2 = (float(v) for v in xyxy[idx])
                class_id = int(classes[idx]) if classes is not None else -1
                name = str(self.class_names.get(class_id, class_id))
                det = Detection2D()
                det.header = msg.header
                det.id = name
                det.bbox.center.position.x = (x1 + x2) * 0.5
                det.bbox.center.position.y = (y1 + y2) * 0.5
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = name
                hypothesis.hypothesis.score = (
                    float(confs[idx]) if confs is not None else 1.0)
                det.results.append(hypothesis)
                msg.detections.append(det)
        self.detections_pub.publish(msg)

    # The debug views are only built for publishers someone subscribes to:
    # opening rqt_image_view on either topic shows the detections right away,
    # and nobody watching costs nothing. circle_set and ball_center are
    # published regardless.
    def _watched(self, pub):
        return pub is not None and pub.get_subscription_count() > 0

    def _publish_annotated(self, results, cv_img, header):
        if not (self._watched(self.image_pub)
                or self._watched(self.image_jpeg_pub)):
            return
        try:
            annotated = results[0].plot() if results else cv_img
        except Exception:  # noqa: BLE001
            annotated = cv_img
        self._publish_image(annotated, header)

    def _publish_image(self, cv_img, header):
        try:
            if self._watched(self.image_pub):
                out = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
                out.header.stamp = header.stamp
                out.header.frame_id = self.detection_frame_id
                self.image_pub.publish(out)
            if self._watched(self.image_jpeg_pub):
                ok, buf = cv2.imencode(
                    '.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                if ok:
                    out = CompressedImage()
                    out.header.stamp = header.stamp
                    out.header.frame_id = self.detection_frame_id
                    out.format = 'jpeg'
                    # array.array, not bytes: the generated setter validates a
                    # plain sequence byte by byte in Python (~2 ms per frame).
                    out.data = array.array('B', buf.tobytes())
                    self.image_jpeg_pub.publish(out)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('failed to publish image_out: %s' % exc,
                                   throttle_duration_sec=2.0)

    # ---------------------------------------------------------------------
    # FPS reporting
    # ---------------------------------------------------------------------
    def _log_fps(self, detected):
        now = time.monotonic()
        if not self.fps.due(now):
            return
        pipeline_fps, infer_ms, cam_fps = self.fps.take_window(now)
        self.fps_pub.publish(Float32(data=float(pipeline_fps)))
        if infer_ms <= 0.0:
            # Nothing went through the model. Idle-because-disabled is normal;
            # idle with detection on means the camera stopped feeding us.
            if not self.enabled:
                self.get_logger().info(
                    'FPS: 0.0 - detection disabled (camera %.1f FPS)' % cam_fps)
            else:
                self.get_logger().warn(
                    'FPS: 0.0 - no frame processed in the last %.1fs '
                    '(camera publishing on image_in?)' % self.fps.window_sec)
            return
        self.get_logger().info(
            'FPS: %.1f  |  infer %.1f ms (%.1f FPS model-only)  |  '
            'camera %.1f FPS  |  ball: %d'
            % (pipeline_fps, infer_ms, 1000.0 / infer_ms, cam_fps, detected))
        self._warn_if_slow(infer_ms, 'inferensi %.0f dtk terakhir' % self.fps.window_sec)

    def _warn_if_slow(self, infer_ms, sumber):
        if self.slow_infer_warn_ms > 0.0 and infer_ms > self.slow_infer_warn_ms:
            self.get_logger().warn(
                'CPU LAMBAT: %s %.1f ms > %.0f ms (normal ~21 ms). CPU sedang '
                'dibatasi (cek baterai/daya) atau direbut proses lain (build, '
                'browser) -- respon robot akan lambat, jangan uji dulu.'
                % (sumber, infer_ms, self.slow_infer_warn_ms))

    def log_fps_summary(self):
        if self._summary_logged:
            return
        self._summary_logged = True
        # Ctrl-C tears the ROS context down before this runs, and logging
        # through rosout then fails once per line - fall back to stdout.
        emit = self.get_logger().info if rclpy.ok() else print
        for line in self.fps.summary_lines('ball detector'):
            emit(line)


def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.log_fps_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
