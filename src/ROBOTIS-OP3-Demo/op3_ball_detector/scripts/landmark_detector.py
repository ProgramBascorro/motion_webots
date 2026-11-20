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

"""Field-landmark detector running the weights in ``model/landmark_deploy``.

This is a standalone program: it detects field landmarks and nothing else. The
ball is handled by a separate program, ``ball_detector.py``, so the two can be
started, stopped and tuned independently.

The model knows five classes, straight from landmark_export.yaml:

    L, T, X          line intersections
    goalpost
    center_circle

Detections go out as ``vision_msgs/Detection2DArray`` - the standard 2D
detection message, already available in this container - with pixel bounding
boxes, the class name in both ``id`` and ``results[0].hypothesis.class_id``,
and the score in ``results[0].hypothesis.score``.

Topics (relative, so a namespace moves them all together):

    image_in   (sensor_msgs/CompressedImage or Image)  camera in
    enable     (std_msgs/Bool)                         gate detection
    detections (vision_msgs/Detection2DArray)         the landmarks
    fps        (std_msgs/Float32)                     pipeline FPS, live
    image_out  (sensor_msgs/Image)                     annotated debug view

Weights are picked automatically from ``model/landmark_deploy`` to match the
runtimes actually installed here: the OpenVINO IR directory when the openvino
package is present (the robot container - the int8 export is the fast one),
the .onnx when onnxruntime is present, the .pt otherwise. Set the
``model_path`` parameter to override.

FPS is reported continuously: every ``fps_log_period`` seconds while running,
and once more as a summary when the node stops.

Run:
    ros2 launch op3_ball_detector landmark_detector.launch.py
    ros2 run op3_ball_detector landmark_detector.py --ros-args -p device:=cpu
"""

import glob
import importlib.util
import os
import threading
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

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


class LandmarkDetector(Node):
    """Detects field landmarks with the model/landmark_deploy weights."""

    # Where this program's weights live, relative to the package root.
    MODEL_DIR = os.path.join('model', 'landmark_deploy')

    def __init__(self):
        super().__init__('landmark_detector')

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
        # Landmarks are small and often partly occluded, so the default sits
        # lower than the ball detector's.
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.4).value)
        self.iou_threshold = float(
            self.declare_parameter('iou_threshold', 0.45).value)
        # Must match the imgsz the weights were exported with (640 here).
        self.imgsz = int(self.declare_parameter('imgsz', 640).value)
        self.device = str(self.declare_parameter('device', 'cpu').value)
        self.half = bool(self.declare_parameter('half', False).value)
        # A field view can hold many intersections at once.
        self.max_detections = int(
            self.declare_parameter('max_detections', 30).value)
        # Which of the model's classes to keep. Listing all five = keep
        # everything; drop names here to ignore a class entirely.
        self.target_classes = list(
            self.declare_parameter(
                'target_classes',
                ['L', 'T', 'X', 'goalpost', 'center_circle']).value)
        self.detection_rate = float(
            self.declare_parameter('detection_rate', 15.0).value)
        self.publish_image = bool(
            self.declare_parameter('publish_image', True).value)
        self.detection_frame_id = str(
            self.declare_parameter('detection_frame_id', 'detector').value)
        self.enabled = bool(
            self.declare_parameter('enable_at_start', True).value)
        # How often the FPS line is logged, in seconds.
        self.fps_log_period = float(
            self.declare_parameter('fps_log_period', 5.0).value)

        if self.detection_rate <= 0.0:
            self.get_logger().warn('detection_rate <= 0, defaulting to 15 Hz')
            self.detection_rate = 15.0

        self.fps = FpsMeter(self.fps_log_period)
        self._class_totals = Counter()   # class name -> detections over the run
        self._summary_logged = False

        # ----- model ------------------------------------------------------
        self.model = self._load_model()

        # ----- ROS interface ----------------------------------------------
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_image = None        # (cv_bgr, header)

        self.detections_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        # The same number the FPS log line carries, for rqt_plot and friends.
        self.fps_pub = self.create_publisher(Float32, 'fps', 1)
        self.image_pub = (self.create_publisher(Image, 'image_out', 1)
                          if self.publish_image else None)

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

        # Inference runs on the freshest frame at a fixed rate. Decoupling it
        # from the camera rate stops a queue building up when YOLO is slower
        # than the camera.
        self.timer = self.create_timer(
            1.0 / self.detection_rate, self._process_latest)

        self.get_logger().info(
            'landmark detector ready (compressed=%s, conf=%.2f, imgsz=%d, '
            'device=%s, rate=%.1fHz, enabled=%s, fps log every %.1fs)'
            % (self.use_compressed, self.confidence_threshold, self.imgsz,
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
        self.get_logger().info(
            'loading landmark weights (%s): %s' % (backend, path))
        model = (YOLO(path, task=self.model_task)
                 if self.model_task else YOLO(path))

        self.class_names = dict(model.names)
        self.target_class_ids = self._resolve_target_class_ids()
        self.get_logger().info(
            'model classes: %s | targeting ids: %s'
            % (self.class_names, self.target_class_ids))

        self._warmup(model)
        return model

    def _warmup(self, model):
        """Run a few dummy frames so the first real one is not penalised.

        The timing is also the first FPS number of the run: it says what the
        weights can do here before the camera is even involved.
        """
        try:
            import numpy as np
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            model.predict(dummy, imgsz=self.imgsz, device=self.device,
                          half=self.half, verbose=False)
            start = time.monotonic()
            runs = 3
            for _ in range(runs):
                model.predict(dummy, imgsz=self.imgsz, device=self.device,
                              half=self.half, verbose=False)
            per_frame = (time.monotonic() - start) / runs
            self.get_logger().info(
                'warm-up: %.1f ms/frame -> %.1f FPS model-only (pipeline is '
                'capped at detection_rate=%.1f FPS)'
                % (per_frame * 1000.0, 1.0 / max(per_frame, 1e-9),
                   self.detection_rate))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('warm-up skipped: %s' % exc)

    def _resolve_target_class_ids(self):
        """Map configured class names to model class indices.

        None means "keep every class", which is also the fallback when nothing
        matches - better a noisy detector than a silent one.
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
                'no configured target class matched the model; '
                'falling back to detecting ALL classes.')
            return None
        if len(ids) == len(self.class_names):
            return None       # everything is wanted; skip the filter entirely
        return ids

    # ---------------------------------------------------------------------
    # subscriptions
    # ---------------------------------------------------------------------
    def _enable_callback(self, msg: Bool):
        if msg.data != self.enabled:
            self.get_logger().info(
                'detection %s' % ('ENABLED' if msg.data else 'DISABLED'))
        self.enabled = msg.data

    def _compressed_image_callback(self, msg: CompressedImage):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('compressed cv_bridge failed: %s' % exc)
            return
        self.fps.count_input()
        with self._frame_lock:
            self._latest_image = (cv_img, msg.header)

    def _image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge failed: %s' % exc)
            return
        self.fps.count_input()
        with self._frame_lock:
            self._latest_image = (cv_img, msg.header)

    # ---------------------------------------------------------------------
    # detection loop
    # ---------------------------------------------------------------------
    def _process_latest(self):
        with self._frame_lock:
            data = self._latest_image
            self._latest_image = None

        counts = Counter()
        if data is not None:
            cv_img, header = data
            if self.enabled:
                counts = self._detect_and_publish(cv_img, header)
            else:
                # No detections while disabled, but keep streaming the raw
                # image so the debug view does not freeze.
                self._publish_image(cv_img, header)

        self._log_fps(counts)

    def _detect_and_publish(self, cv_img, header):
        try:
            start = time.monotonic()
            results = self.model.predict(
                cv_img,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
                half=self.half,
                max_det=self.max_detections,
                classes=self.target_class_ids,
                verbose=False)
            self.fps.count_frame(time.monotonic() - start)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('inference failed: %s' % exc,
                                    throttle_duration_sec=2.0)
            return Counter()

        detections = self._extract_detections(results)
        self._publish_detections(detections, header)
        self._publish_annotated(results, cv_img, header)

        counts = Counter(d[0] for d in detections)
        self._class_totals.update(counts)
        return counts

    def _extract_detections(self, results):
        """YOLO boxes -> (class_name, score, cx, cy, w, h) tuples, px units."""
        detections = []
        if not results:
            return detections
        boxes = getattr(results[0], 'boxes', None)
        if boxes is None or boxes.xyxy is None:
            return detections

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy() if boxes.conf is not None else None
        classes = boxes.cls.cpu().numpy() if boxes.cls is not None else None
        for idx, (x1, y1, x2, y2) in enumerate(xyxy):
            class_id = int(classes[idx]) if classes is not None else -1
            name = str(self.class_names.get(class_id, class_id))
            score = float(confs[idx]) if confs is not None else 1.0
            detections.append((name, score,
                               float((x1 + x2) * 0.5), float((y1 + y2) * 0.5),
                               float(x2 - x1), float(y2 - y1)))

        # Most confident first, so a consumer that only wants one can take [0].
        detections.sort(key=lambda d: d[1], reverse=True)
        return detections

    def _publish_detections(self, detections, header):
        msg = Detection2DArray()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.detection_frame_id
        for name, score, cx, cy, w, h in detections:
            det = Detection2D()
            det.header = msg.header
            det.id = name
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = name
            hypothesis.hypothesis.score = score
            det.results.append(hypothesis)
            msg.detections.append(det)
        self.detections_pub.publish(msg)

    def _publish_annotated(self, results, cv_img, header):
        if self.image_pub is None:
            return
        try:
            annotated = results[0].plot() if results else cv_img
        except Exception:  # noqa: BLE001
            annotated = cv_img
        self._publish_image(annotated, header)

    def _publish_image(self, cv_img, header):
        if self.image_pub is None:
            return
        try:
            out = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
            out.header.stamp = header.stamp
            out.header.frame_id = self.detection_frame_id
            self.image_pub.publish(out)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('failed to publish image_out: %s' % exc,
                                   throttle_duration_sec=2.0)

    # ---------------------------------------------------------------------
    # FPS reporting
    # ---------------------------------------------------------------------
    @staticmethod
    def _format_counts(counts):
        if not counts:
            return 'none'
        return ' '.join('%s=%d' % (name, n) for name, n in sorted(counts.items()))

    def _log_fps(self, counts):
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
            'camera %.1f FPS  |  landmarks: %s'
            % (pipeline_fps, infer_ms, 1000.0 / infer_ms, cam_fps,
               self._format_counts(counts)))

    def log_fps_summary(self):
        if self._summary_logged:
            return
        self._summary_logged = True
        # Ctrl-C tears the ROS context down before this runs, and logging
        # through rosout then fails once per line - fall back to stdout.
        emit = self.get_logger().info if rclpy.ok() else print
        for line in self.fps.summary_lines('landmark detector'):
            emit(line)
        if self.fps.total_frames:
            emit('detections total   : %s'
                 % self._format_counts(self._class_totals))


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkDetector()
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
