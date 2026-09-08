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

"""Field-landmark detector for ROBOTIS OP3, running an OpenVINO model.

Runs the INT8 OpenVINO export that ships in
``model/landmark_deploy/best_landmark_v8n_int8_openvino_model`` and publishes
EVERY landmark it sees, with its class label and score:

    0: L   1: T   2: X   3: goalpost   4: center_circle

``L``, ``T`` and ``X`` are white-line junctions, ``goalpost`` is a post and
``center_circle`` is the kick-off circle. Together they are what a humanoid
localiser matches against the field model.

This is the RAW landmark feed. Its sibling ``yolo_goalline_detector.py``
consumes the same model but answers one specific geometric question (where is
the goal line); this node stays deliberately unopinionated — it reports what the
model saw and lets the consumer decide.

Published topics (relative to the node namespace, default
``landmark_detector_node``):

    detections   vision_msgs/Detection2DArray  every landmark, IMAGE PIXELS:
                     .id / .results[0].hypothesis.class_id = class name
                     .results[0].hypothesis.score          = confidence
                     .bbox.center.position.x / .y          = centre (px)
                     .bbox.size_x / .size_y                = size (px)
    landmark_count std_msgs/Int32              landmarks in this frame
    fps          std_msgs/Float32              live processing rate
    image_out    sensor_msgs/Image             annotated debug image

Subscribed:

    image_in     sensor_msgs/CompressedImage or Image (see use_compressed)
    enable       std_msgs/Bool                 gate detection on/off

FPS REPORTING — in FPS, never as a bare millisecond figure:
  * at start-up the model is benchmarked and its ceiling logged,
  * every ``fps_log_period`` seconds a live FPS line is logged (with the
    per-class tally of what is on screen) and the rate published on ``fps``,
  * on shutdown a summary block reports the average/min/max FPS, the
    millisecond breakdown behind it, and how often each class was seen.
"""

import collections
import os
import signal
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool, Float32, Int32
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from cv_bridge import CvBridge

# detector_fps.py is installed into the SAME directory as this script
# (lib/op3_ball_detector), and sits next to it in scripts/ when run from source.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detector_fps import FpsMeter  # noqa: E402
from goalline_core import draw_hud, extract_detections, speed_of  # noqa: E402


class YoloLandmarkDetector(Node):
    """Detects field landmarks with an OpenVINO YOLO model."""

    def __init__(self):
        super().__init__('yolo_landmark_detector')

        # ----- model / inference parameters -------------------------------
        self.model_path = str(self.declare_parameter(
            'model_path',
            'model/landmark_deploy/best_landmark_v8n_int8_openvino_model').value)
        self.model_task = str(
            self.declare_parameter('model_task', 'detect').value).strip()
        self.use_compressed = bool(
            self.declare_parameter('use_compressed', True).value)
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.5).value)
        self.iou_threshold = float(
            self.declare_parameter('iou_threshold', 0.45).value)
        # MUST match the exported size: this IR is a static [1,3,640,640].
        self.imgsz = int(self.declare_parameter('imgsz', 640).value)
        self.device = str(self.declare_parameter('device', 'cpu').value)
        self.half = bool(self.declare_parameter('half', False).value)
        # Ultralytics deprecated the predict-time 'half' arg: passing it at all
        # (even =False) warns on every call. Only forward it when enabled.
        self._predict_precision = {'half': True} if self.half else {}
        self.max_detections = int(
            self.declare_parameter('max_detections', 30).value)
        self.detection_rate = float(
            self.declare_parameter('detection_rate', 10.0).value)

        # Subset of class names to keep. EMPTY = every class the model exposes.
        # NOTE: an empty array ("target_classes: []") comes back as None because
        # ROS 2 cannot infer the element type of an empty list, so guard it.
        _target_classes = self.declare_parameter('target_classes', []).value
        self.target_classes = list(_target_classes) if _target_classes else []

        # ----- output / reporting parameters -------------------------------
        self.publish_image = bool(
            self.declare_parameter('publish_image', True).value)
        self.detection_frame_id = str(
            self.declare_parameter('detection_frame_id', 'detector').value)
        self.enabled = bool(
            self.declare_parameter('enable_at_start', True).value)
        self.fps_log_period = float(
            self.declare_parameter('fps_log_period', 5.0).value)
        self.fps_window_sec = float(
            self.declare_parameter('fps_window_sec', 2.0).value)
        self.startup_benchmark_frames = int(
            self.declare_parameter('startup_benchmark_frames', 5).value)

        if self.detection_rate <= 0.0:
            self.get_logger().warn('detection_rate <= 0, defaulting to 10 Hz')
            self.detection_rate = 10.0

        # ----- state -------------------------------------------------------
        # Keep the label <= 16 chars: summary_lines pads it to that width, and
        # a longer one pushes its column out of line with the rows above.
        self.fps = FpsMeter(window_sec=self.fps_window_sec, name='landmark',
                            hit_label='landmark seen')
        # How many times each class was seen over the whole run, and in how many
        # frames it appeared at all — the two numbers that tell you whether a
        # landmark model is usable on this field.
        self._class_totals = collections.Counter()
        self._class_frames = collections.Counter()
        self._summary_logged = False
        self._last_fps_log = time.monotonic()

        self.model = self._load_model()

        # ----- ROS interface -------------------------------------------------
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_image = None        # (cv_bgr, header)

        self.detections_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        self.count_pub = self.create_publisher(Int32, 'landmark_count', 10)
        self.fps_pub = self.create_publisher(Float32, 'fps', 10)
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

        # ----- image watchdog ------------------------------------------------
        # A detector that receives nothing must SAY so. Without this the node
        # sits perfectly silent when no camera is publishing, which on screen is
        # indistinguishable from "the model found nothing" — and the only clue
        # is the absence of lines that were never promised.
        self._frames_in = 0
        self._last_frame_time = None
        self._start_time = time.monotonic()
        self.watchdog = self.create_timer(1.0, self._check_image_flow)

        # Inference runs on the FRESHEST frame at a fixed rate: decoupling the
        # camera rate from inference stops a slow model building a queue.
        self.timer = self.create_timer(
            1.0 / self.detection_rate, self._process_latest)

        self.get_logger().info(
            'Listening for images on "%s"'
            % self.image_sub.topic_name)
        self.get_logger().info(
            'YOLO landmark detector ready '
            '(compressed=%s, conf=%.2f, imgsz=%d, device=%s, rate=%.1fHz, '
            'enabled=%s)'
            % (self.use_compressed, self.confidence_threshold, self.imgsz,
               self.device, self.detection_rate, self.enabled))

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

        model_path = self.model_path.strip()
        if not model_path:
            self.get_logger().fatal(
                'model_path is empty. Point it at the landmark OpenVINO dir, '
                'e.g. model/landmark_deploy/best_landmark_v8n_int8_openvino_model')
            raise ValueError('model_path is required')
        if not os.path.isabs(model_path) and not os.path.exists(model_path):
            # A relative path resolves against the package share dir, where
            # install(DIRECTORY ... model) puts the weights.
            model_path = os.path.join(
                get_package_share_directory('op3_ball_detector'), model_path)
        if not os.path.exists(model_path):
            self.get_logger().fatal('Model path not found: %s' % model_path)
            raise FileNotFoundError(model_path)

        # An OpenVINO model is a DIRECTORY holding .xml + .bin. Half of that
        # pair going missing is the classic way this fails, and ultralytics'
        # own error for it is unhelpful, so say it plainly here.
        if os.path.isdir(model_path):
            have = os.listdir(model_path)
            for suffix in ('.xml', '.bin'):
                if not any(f.endswith(suffix) for f in have):
                    self.get_logger().fatal(
                        'OpenVINO dir %s has no %s file (found: %s). Re-extract '
                        'the export — the IR needs BOTH .xml and .bin.'
                        % (model_path, suffix, sorted(have)))
                    raise FileNotFoundError(os.path.join(model_path, '*' + suffix))

        self.get_logger().info('Loading YOLO model: %s' % model_path)
        model = (YOLO(model_path, task=self.model_task)
                 if self.model_task else YOLO(model_path))
        # Warm up BEFORE touching model.names. On a non-.pt model (OpenVINO
        # here) ultralytics has to BUILD A PREDICTOR just to answer .names, and
        # predict() then builds its own — so reading the names first loads the
        # model TWICE at start-up. It shows in the log as two "Loading ... for
        # OpenVINO inference..." / "Using OpenVINO LATENCY mode ..." pairs.
        # With the warm-up first, the predictor already exists and .names just
        # reads it.
        self._benchmark_model(model)

        self.class_names = dict(model.names)
        self.target_class_ids = self._resolve_target_class_ids()
        self.get_logger().info(
            'Model classes: %s | keeping: %s'
            % (self.class_names,
               [self.class_names[i] for i in self.target_class_ids]
               if self.target_class_ids else 'ALL'))
        return model

    def _resolve_target_class_ids(self):
        """Map configured class names to model indices. None = keep all."""
        if not self.target_classes:
            return None
        name_to_id = {str(name).lower(): idx
                      for idx, name in self.class_names.items()}
        ids = []
        for wanted in self.target_classes:
            key = str(wanted).strip().lower()
            if key in name_to_id:
                ids.append(name_to_id[key])
            else:
                self.get_logger().warn(
                    'target class "%s" not in model; ignoring' % wanted)
        if not ids:
            self.get_logger().warn(
                'No configured target class matched the model; keeping ALL.')
            return None
        return ids

    def _benchmark_model(self, model):
        """Warm the model up AND report the FPS ceiling it reaches here.

        The first inference pays for lazy IR compilation (~200 ms on OpenVINO
        against ~20 ms steady state), so it is thrown away rather than timed.
        What is left is an upper bound: the loop can never beat it, and a
        detection_rate above it is wishful thinking.
        """
        frames = max(1, self.startup_benchmark_frames)
        dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        try:
            model.predict(dummy, imgsz=self.imgsz, device=self.device,
                          verbose=False, **self._predict_precision)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('Model warm-up skipped: %s' % exc)
            return

        started = time.monotonic()
        for _ in range(frames):
            model.predict(dummy, imgsz=self.imgsz, device=self.device,
                          verbose=False, **self._predict_precision)
        elapsed = time.monotonic() - started
        if elapsed <= 0.0:
            return
        ceiling = frames / elapsed
        note = ''
        if ceiling < self.detection_rate:
            note = (' — BELOW the requested detection_rate of %.1f Hz, so the '
                    'loop will run at about %.1f FPS'
                    % (self.detection_rate, ceiling))
        self.get_logger().info(
            'Model ceiling on this machine: %.1f FPS (%.1f ms/frame over %d '
            'warm frames)%s'
            % (ceiling, elapsed / frames * 1000.0, frames, note))

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
        self._note_incoming_frame(cv_img)
        with self._frame_lock:
            self._latest_image = (cv_img, msg.header)

    def _image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('cv_bridge failed: %s' % exc)
            return
        self._note_incoming_frame(cv_img)
        with self._frame_lock:
            self._latest_image = (cv_img, msg.header)

    def _note_incoming_frame(self, cv_img):
        """Book-keeping for the watchdog, called from the image callbacks."""
        self._frames_in += 1
        self._last_frame_time = time.monotonic()
        if self._frames_in == 1:
            height, width = cv_img.shape[:2]
            self.get_logger().info(
                'First image received on "%s": %dx%d (mean level %.1f/255)'
                % (self.image_sub.topic_name, width, height,
                   self._frame_level(cv_img)))
            return

        # A frame that is essentially black — lens covered, lights off, head
        # pointed into a shadow — yields zero detections and on the console
        # looks EXACTLY like a model that cannot see anything. Say which of the
        # two it is. Sampled every 8th pixel, so this costs nothing.
        level = self._frame_level(cv_img)
        if level < 16.0:
            self.get_logger().warn(
                'Image on "%s" is almost BLACK (mean %.1f/255): there is '
                'nothing in it for the model to find. Check the lens cap, the '
                'room lights, and where the head is pointing.'
                % (self.image_sub.topic_name, level),
                throttle_duration_sec=10.0)
        elif level > 245.0:
            self.get_logger().warn(
                'Image on "%s" is BLOWN OUT (mean %.1f/255): overexposed, so '
                'the model has no contrast to work with.'
                % (self.image_sub.topic_name, level),
                throttle_duration_sec=10.0)

    @staticmethod
    def _frame_level(cv_img):
        """Mean brightness over a cheap subsample of the frame (0..255)."""
        return float(cv_img[::8, ::8].mean())

    def _check_image_flow(self):
        """Say out loud when nothing is arriving on image_in."""
        now = time.monotonic()
        topic = self.image_sub.topic_name
        if self._frames_in == 0:
            # Give the camera a moment to come up before complaining.
            if (now - self._start_time) < 3.0:
                return
            self.get_logger().warn(
                'NO IMAGE on "%s" after %.0f s — nothing is publishing there, '
                'so this detector has nothing to look at. Either start the '
                'camera together with it (use_usb_cam:=true), or point '
                'image_topic:= at a topic that has a publisher. Check with: '
                'ros2 topic hz %s'
                % (topic, now - self._start_time, topic),
                throttle_duration_sec=5.0)
            return
        gap = now - self._last_frame_time
        if gap > 3.0:
            self.get_logger().warn(
                'Image flow STOPPED on "%s": nothing for %.1f s after %d '
                'frames. Did the camera node die?'
                % (topic, gap, self._frames_in),
                throttle_duration_sec=5.0)

    # ---------------------------------------------------------------------
    # main detection loop
    # ---------------------------------------------------------------------
    def _process_latest(self):
        # Ctrl-C tears the context down while this callback may be halfway
        # through an inference; every publish() after that raises RCLError
        # ("publisher's context is invalid"). That is a clean shutdown, not a
        # failure. rclpy Humble exposes no importable RCLError class, hence the
        # rclpy.ok() test rather than a narrower "except".
        if not rclpy.ok():
            return
        try:
            self._process_frame()
        except Exception:  # noqa: BLE001
            if not rclpy.ok():
                return
            raise

    def _process_frame(self):
        with self._frame_lock:
            data = self._latest_image
            self._latest_image = None
        if data is None:
            return
        cv_img, header = data
        loop_start = time.monotonic()

        if not self.enabled:
            self._publish_detections([], header)
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

        detections = extract_detections(results, self.class_names)
        self._publish_detections(detections, header)

        per_class = collections.Counter(det[4] for det in detections)
        self._class_totals.update(per_class)
        self._class_frames.update(per_class.keys())

        # ----- FPS accounting -------------------------------------------
        pre_ms, infer_ms, post_ms = speed_of(results)
        loop_ms = (time.monotonic() - loop_start) * 1000.0
        fps_now = self.fps.tick(now=loop_start, infer_ms=infer_ms,
                                pre_ms=pre_ms, post_ms=post_ms,
                                loop_ms=loop_ms, hit=bool(detections))
        self.fps_pub.publish(Float32(data=float(fps_now)))
        self._log_fps_periodically(per_class)

        self._publish_annotated(results, cv_img, header, fps_now, per_class)

    def _log_fps_periodically(self, per_class):
        now = time.monotonic()
        if (now - self._last_fps_log) < max(0.5, self.fps_log_period):
            return
        self._last_fps_log = now
        seen = (', '.join('%s x%d' % (name, count)
                          for name, count in sorted(per_class.items()))
                if per_class else 'nothing on screen')
        self.get_logger().info('%s | %s' % (self.fps.live_line(), seen))

    # ---------------------------------------------------------------------
    # publishing
    # ---------------------------------------------------------------------
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
        self.count_pub.publish(Int32(data=len(detections)))

    def _publish_annotated(self, results, cv_img, header, fps_now, per_class):
        # Skip plot()/encode/publish entirely when nobody subscribes, so the
        # overlay costs nothing during a match yet stays available on demand.
        if self.image_pub is None or self.image_pub.get_subscription_count() == 0:
            return
        try:
            annotated = results[0].plot() if results else cv_img.copy()
        except Exception:  # noqa: BLE001
            annotated = cv_img.copy()
        seen = (', '.join('%s x%d' % (n, c) for n, c in sorted(per_class.items()))
                if per_class else 'no landmark')
        draw_hud(annotated, ['FPS %.1f (avg %.1f)'
                             % (fps_now, self.fps.average_fps), seen])
        try:
            out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            out.header.stamp = header.stamp
            out.header.frame_id = self.detection_frame_id
            self.image_pub.publish(out)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('failed to publish image_out: %s' % exc,
                                   throttle_duration_sec=2.0)

    # ---------------------------------------------------------------------
    # shutdown report
    # ---------------------------------------------------------------------
    def log_fps_summary(self):
        """Log the end-of-run FPS block. Safe to call more than once."""
        if self._summary_logged:
            return
        self._summary_logged = True

        # After Ctrl-C rclpy's context is already down, and every logger call
        # then also prints "Failed to publish log message to rosout" — which
        # would bury the summary in noise. Fall back to plain stdout there.
        emit = self.get_logger().info if rclpy.ok() else print

        if self.fps.frame_count == 0:
            emit('FPS SUMMARY: no frame was processed — was anything '
                 'publishing on image_in?')
            return

        frames = self.fps.frame_count
        per_class_lines = ['per class        : (name: total boxes / frames seen)']
        for name in sorted(self._class_totals,
                           key=lambda n: -self._class_totals[n]):
            per_class_lines.append(
                '  %-14s : %5d boxes / %5d frames (%.1f%%)'
                % (name, self._class_totals[name], self._class_frames[name],
                   100.0 * self._class_frames[name] / frames))
        if not self._class_totals:
            per_class_lines.append('  (no landmark was detected in this run)')

        for line in self.fps.summary_lines(title='landmark detector',
                                           target_fps=self.detection_rate,
                                           extra_lines=per_class_lines):
            emit(line)


def main(args=None):
    rclpy.init(args=args)
    node = YoloLandmarkDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C under `ros2 launch` delivers SIGINT more than once: the
        # terminal signals the whole foreground process group, then launch
        # forwards one of its own. Those extras used to land in the middle of
        # the teardown and end the run with a traceback instead of the summary.
        # Deafen SIGINT for the entire teardown; SIGTERM/SIGKILL still work.
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except ValueError:
            pass

        # Report the achieved FPS on EVERY exit, Ctrl-C included.
        node.log_fps_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
