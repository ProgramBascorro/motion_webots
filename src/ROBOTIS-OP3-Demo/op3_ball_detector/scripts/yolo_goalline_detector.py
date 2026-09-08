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

"""Goal-line (garis gawang) detector for ROBOTIS OP3.

Runs the best_landmark_v8n model shipped in ``model/landmark_deploy.zip`` — a
5-class YOLOv8n detector (L, T, X, goalpost, center_circle) — and turns its raw
boxes into ONE geometric answer: where the goal line is in the image.

    goal line = the line through the FEET of the goalposts, refined with any
                L/T/X line junction that already lies on that same line.

Published topics (all relative to the node namespace, default
``goalline_detector_node``):

    detections   vision_msgs/Detection2DArray  every landmark box + class + score
    goal_line    geometry_msgs/PolygonStamped  4 points, IMAGE PIXELS:
                     [0] segment end A      [1] segment end B
                     [2] goalpost foot A    [3] goalpost foot B
                 (points 2/3 are omitted when only one post is visible)
    goal_center  geometry_msgs/PointStamped   x,y NORMALIZED to [-1, 1] like the
                 ball detector (-1 = left/top, +1 = right/bottom); z = the goal
                 width in pixels
    goal_found   std_msgs/Bool                is the estimate valid this frame
    fps          std_msgs/Float32             live processing rate
    image_out    sensor_msgs/Image            annotated debug image

Subscribed:

    image_in     sensor_msgs/CompressedImage or Image (see use_compressed)
    enable       std_msgs/Bool                gate detection on/off

FPS REPORTING — every run tells you what it got:
  * at start-up the model is benchmarked on a few blank frames and the ceiling
    is logged ("model ceiling: NN.N FPS"),
  * every ``fps_log_period`` seconds a live line is logged and the rate is
    published on ``fps``,
  * on shutdown (Ctrl-C included) a summary block is logged with the average,
    min/max and the pre/infer/post millisecond breakdown.

This node is INDEPENDENT of the ball pipeline; it is meant to run alongside
yolo_ball_detector.py, sharing the same camera topic.
"""

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

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from cv_bridge import CvBridge

# goalline_core.py is installed into the SAME directory as this script
# (lib/op3_ball_detector), and sits next to it in scripts/ when run from source.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detector_fps import FpsMeter  # noqa: E402
from goalline_core import (  # noqa: E402
    GoalLineEstimator,
    draw_goal_line,
    draw_hud,
    extract_detections,
    speed_of,
)


class YoloGoalLineDetector(Node):
    """Detects the goal line with the best_landmark_v8n landmark model."""

    def __init__(self):
        super().__init__('yolo_goalline_detector')

        # ----- model / inference parameters -------------------------------
        self.model_path = str(self.declare_parameter(
            'model_path', 'model/best_landmark_v8n_openvino_model').value)
        self.model_task = str(
            self.declare_parameter('model_task', 'detect').value).strip()
        self.use_compressed = bool(
            self.declare_parameter('use_compressed', True).value)
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.5).value)
        self.iou_threshold = float(
            self.declare_parameter('iou_threshold', 0.45).value)
        # MUST match the imgsz the model was exported with: best_landmark_v8n's
        # OpenVINO IR is a static [1,3,640,640], so 640 here.
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

        # ----- goal-line geometry parameters -------------------------------
        self.min_goalposts = int(
            self.declare_parameter('min_goalposts', 2).value)
        self.use_landmarks = bool(
            self.declare_parameter('use_landmarks', True).value)
        self.landmark_band_px = float(
            self.declare_parameter('landmark_band_px', 40.0).value)
        self.min_linearity = float(
            self.declare_parameter('min_linearity', 0.90).value)
        self.min_length_px = float(
            self.declare_parameter('min_length_px', 30.0).value)
        self.smoothing = float(self.declare_parameter('smoothing', 0.5).value)
        self.hold_sec = float(self.declare_parameter('hold_sec', 0.4).value)

        # ----- output / reporting parameters -------------------------------
        self.publish_image = bool(
            self.declare_parameter('publish_image', True).value)
        self.draw_boxes = bool(self.declare_parameter('draw_boxes', True).value)
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
        self.fps = FpsMeter(window_sec=self.fps_window_sec, name='goalline',
                            hit_label='goal line found')
        self.estimator = GoalLineEstimator(
            min_goalposts=self.min_goalposts,
            use_landmarks=self.use_landmarks,
            landmark_band_px=self.landmark_band_px,
            min_linearity=self.min_linearity,
            min_length_px=self.min_length_px,
            smoothing=self.smoothing,
            hold_sec=self.hold_sec)
        self._summary_logged = False
        self._last_fps_log = time.monotonic()

        self.model = self._load_model()

        # ----- ROS interface -------------------------------------------------
        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_image = None        # (cv_bgr, header)

        self.detections_pub = self.create_publisher(
            Detection2DArray, 'detections', 10)
        self.goal_line_pub = self.create_publisher(
            PolygonStamped, 'goal_line', 10)
        self.goal_center_pub = self.create_publisher(
            PointStamped, 'goal_center', 10)
        self.goal_found_pub = self.create_publisher(Bool, 'goal_found', 10)
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
            'YOLO goal-line detector ready '
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
                'model_path is empty. Point it at the landmark OpenVINO model '
                'dir, e.g. model/best_landmark_v8n_openvino_model')
            raise ValueError('model_path is required')
        if not os.path.isabs(model_path) and not os.path.exists(model_path):
            # A relative path is resolved against the package share dir, where
            # install(DIRECTORY ... model) puts the weights.
            model_path = os.path.join(
                get_package_share_directory('op3_ball_detector'), model_path)
        if not os.path.exists(model_path):
            self.get_logger().fatal('Model path not found: %s' % model_path)
            raise FileNotFoundError(model_path)

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
        self.get_logger().info('Model classes: %s' % self.class_names)

        expected = {'goalpost', 'l', 't', 'x'}
        present = {str(n).lower() for n in self.class_names.values()}
        if 'goalpost' not in present:
            self.get_logger().error(
                'This model has no "goalpost" class (%s) — the goal line '
                'cannot be reconstructed from it.' % sorted(present))
        elif not expected.issubset(present):
            self.get_logger().warn(
                'Model is missing some line-junction classes %s; the goal line '
                'will rest on the goalposts alone.'
                % sorted(expected - present))
        return model

    def _benchmark_model(self, model):
        """Warm the model up AND report the FPS ceiling it can reach here.

        The first inference pays for lazy graph/IR init, so it is discarded. The
        rest give an upper bound: the loop can never be faster than this, and
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
        per_frame_ms = elapsed / frames * 1000.0
        ceiling = frames / elapsed
        note = ''
        if ceiling < self.detection_rate:
            note = (' — BELOW the requested detection_rate of %.1f Hz, so the '
                    'loop will run at about %.1f FPS'
                    % (self.detection_rate, ceiling))
        self.get_logger().info(
            'Model ceiling on this machine: %.1f FPS (%.1f ms/frame over %d '
            'warm frames)%s' % (ceiling, per_frame_ms, frames, note))

    # ---------------------------------------------------------------------
    # subscriptions
    # ---------------------------------------------------------------------
    def _enable_callback(self, msg: Bool):
        if msg.data != self.enabled:
            self.get_logger().info(
                'Detection %s' % ('ENABLED' if msg.data else 'DISABLED'))
            if not msg.data:
                self.estimator.reset()
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
        # through a 50 ms inference; every publish() after that raises RCLError
        # ("publisher's context is invalid"). That is a clean shutdown, not a
        # failure, so swallow it instead of dumping a traceback over the FPS
        # summary. rclpy Humble exposes no importable RCLError class, hence the
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
            self.estimator.reset()
            self._publish_detections([], header)
            self._publish_goal(None, header, cv_img.shape[1], cv_img.shape[0])
            return

        try:
            results = self.model.predict(
                cv_img,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
                max_det=self.max_detections,
                verbose=False,
                **self._predict_precision)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('YOLO inference failed: %s' % exc,
                                    throttle_duration_sec=2.0)
            return

        detections = extract_detections(results, self.class_names)
        height, width = cv_img.shape[:2]
        goal = self.estimator.update(detections, now=loop_start)

        self._publish_detections(detections, header)
        self._publish_goal(goal, header, width, height)

        # ----- FPS accounting: the whole point of the second requirement ----
        pre_ms, infer_ms, post_ms = speed_of(results)
        loop_ms = (time.monotonic() - loop_start) * 1000.0
        fps_now = self.fps.tick(now=loop_start, infer_ms=infer_ms,
                                pre_ms=pre_ms, post_ms=post_ms,
                                loop_ms=loop_ms, hit=goal is not None)
        self.fps_pub.publish(Float32(data=float(fps_now)))
        self._log_fps_periodically(goal)

        self._publish_annotated(results, cv_img, header, goal, fps_now)

    def _log_fps_periodically(self, goal):
        now = time.monotonic()
        if (now - self._last_fps_log) < max(0.5, self.fps_log_period):
            return
        self._last_fps_log = now
        state = goal.describe() if goal is not None else (
            'no goal line (%s)' % (self.estimator.last_reject or 'nothing seen'))
        self.get_logger().info('%s | %s' % (self.fps.live_line(), state))

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

    def _publish_goal(self, goal, header, width, height):
        found = Bool()
        found.data = goal is not None
        self.goal_found_pub.publish(found)

        poly = PolygonStamped()
        poly.header.stamp = header.stamp
        poly.header.frame_id = self.detection_frame_id
        if goal is not None:
            points = [goal.p1, goal.p2]
            if goal.post_a is not None and goal.post_b is not None:
                points += [goal.post_a, goal.post_b]
            for x, y in points:
                poly.polygon.points.append(
                    Point32(x=float(x), y=float(y), z=0.0))
        self.goal_line_pub.publish(poly)

        if goal is None:
            return
        norm_x, norm_y = goal.center_norm(width, height)
        centre = PointStamped()
        centre.header.stamp = header.stamp
        centre.header.frame_id = self.detection_frame_id
        centre.point.x = float(norm_x)
        centre.point.y = float(norm_y)
        centre.point.z = float(goal.width_px)
        self.goal_center_pub.publish(centre)

    def _publish_annotated(self, results, cv_img, header, goal, fps_now):
        # Skip plot()/encode/publish entirely when nobody subscribes, so the
        # overlay costs nothing during a match yet stays available on demand.
        if self.image_pub is None or self.image_pub.get_subscription_count() == 0:
            return
        try:
            annotated = (results[0].plot()
                         if (self.draw_boxes and results) else cv_img.copy())
        except Exception:  # noqa: BLE001
            annotated = cv_img.copy()
        draw_goal_line(annotated, goal)
        draw_hud(annotated, ['FPS %.1f (avg %.1f)'
                             % (fps_now, self.fps.average_fps)])
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
        for line in self.fps.summary_lines(title='goal-line detector',
                                           target_fps=self.detection_rate):
            emit(line)


def main(args=None):
    rclpy.init(args=args)
    node = YoloGoalLineDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C under `ros2 launch` delivers SIGINT more than once: the
        # terminal signals the whole foreground process group, then launch
        # forwards one of its own. Those extras used to land in the middle of
        # the teardown — inside numpy while the report was being formatted, or
        # inside destroy_node() — and ended the run with a traceback instead of
        # the summary. Deafen SIGINT for the entire teardown; the process is
        # already on its way out, and SIGTERM/SIGKILL still work.
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
