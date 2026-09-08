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

"""Shared core for the goal-line (garis gawang) detector.

Two programs import this module and MUST agree on both the geometry and on how
frames-per-second is counted, so all of it lives here exactly once:

    yolo_goalline_detector.py   ROS 2 node, live camera
    goalline_fps_benchmark.py   standalone CLI, video file / camera / images

Both are installed into the SAME directory (lib/op3_ball_detector), so each one
imports this by putting its own directory on sys.path first.

The frame-rate meter the two of them report with lives next door in
``detector_fps.py``, because the ball and landmark detectors count FPS the same
way and must not grow a second implementation of it.

Nothing here imports rclpy — keep it that way, so the benchmark also runs on a
plain Python install with just ultralytics + OpenCV + NumPy.

The model this is built around is best_landmark_v8n (landmark_deploy.zip), a
5-class YOLOv8n detector:

    0: L   1: T   2: X   3: goalpost   4: center_circle

"L", "T" and "X" are field-line junctions, so they are points that lie ON a
white line; "goalpost" is a post, whose BOTTOM edge is where it touches the
ground. The goal line ("garis gawang") is therefore reconstructed as the line
through the two post feet, optionally refined with any L/T/X junction that
already sits on that same line.
"""

import math
import time

import numpy as np


# Class names in best_landmark_v8n. Compared lower-case, so a re-trained model
# using "Goalpost"/"GOALPOST" still matches.
GOALPOST_CLASS = 'goalpost'
LINE_LANDMARK_CLASSES = ('l', 't', 'x')
CENTER_CIRCLE_CLASS = 'center_circle'


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------
def _fit_line(points):
    """Total-least-squares line fit.

    Returns ``(centroid, direction, linearity)`` or ``None``. ``linearity`` is
    ``1 - s2/s1`` from the SVD: 1.0 = perfectly collinear, ~0 = a blob. A least-
    squares y=ax+b fit is NOT usable here — a goal line seen head-on is vertical
    in the image and would blow up.
    """
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 2:
        return None
    centroid = pts.mean(axis=0)
    centred = pts - centroid

    if len(pts) == 2:
        delta = centred[1] - centred[0]
        norm = float(np.hypot(delta[0], delta[1]))
        if norm < 1e-6:
            return None
        return centroid, delta / norm, 1.0

    try:
        _, sv, vt = np.linalg.svd(centred, full_matrices=False)
    except np.linalg.LinAlgError:
        return None
    direction = vt[0]
    if float(np.hypot(direction[0], direction[1])) < 1e-6:
        return None
    linearity = 1.0 - float(sv[1] / sv[0]) if sv[0] > 1e-9 else 0.0
    return centroid, direction, linearity


def _orient(direction):
    """Force a consistent direction sign so smoothing cannot flip the line."""
    dx, dy = float(direction[0]), float(direction[1])
    if dx < 0.0 or (abs(dx) < 1e-9 and dy < 0.0):
        return np.array([-dx, -dy])
    return np.array([dx, dy])


def _order_endpoints(p1, p2):
    """Left-to-right (then top-to-bottom) so p1/p2 never swap between frames."""
    if (p1[0], p1[1]) <= (p2[0], p2[1]):
        return p1, p2
    return p2, p1


class GoalLine:
    """One goal-line estimate, in IMAGE PIXEL coordinates."""

    __slots__ = ('p1', 'p2', 'post_a', 'post_b', 'width_px', 'confidence',
                 'n_posts', 'n_landmarks', 'linearity', 'stale', 'age_sec')

    def __init__(self, p1, p2, post_a=None, post_b=None, width_px=0.0,
                 confidence=0.0, n_posts=0, n_landmarks=0, linearity=1.0,
                 stale=False, age_sec=0.0):
        self.p1 = (float(p1[0]), float(p1[1]))
        self.p2 = (float(p2[0]), float(p2[1]))
        self.post_a = None if post_a is None else (float(post_a[0]), float(post_a[1]))
        self.post_b = None if post_b is None else (float(post_b[0]), float(post_b[1]))
        self.width_px = float(width_px)
        self.confidence = float(confidence)
        self.n_posts = int(n_posts)
        self.n_landmarks = int(n_landmarks)
        self.linearity = float(linearity)
        self.stale = bool(stale)
        self.age_sec = float(age_sec)

    # -- derived ---------------------------------------------------------
    @property
    def center(self):
        return ((self.p1[0] + self.p2[0]) * 0.5,
                (self.p1[1] + self.p2[1]) * 0.5)

    @property
    def length_px(self):
        return float(math.hypot(self.p2[0] - self.p1[0], self.p2[1] - self.p1[1]))

    @property
    def angle_deg(self):
        """Tilt vs. the image horizontal, in (-90, 90].

        Image y grows downwards, so a POSITIVE angle means the line slopes down
        to the right on screen — i.e. the robot is looking at the goal from its
        left-hand side.
        """
        angle = math.degrees(math.atan2(self.p2[1] - self.p1[1],
                                        self.p2[0] - self.p1[0]))
        while angle <= -90.0:
            angle += 180.0
        while angle > 90.0:
            angle -= 180.0
        return angle

    @property
    def goal_center(self):
        """Mid-point between the two posts, or the segment centre if only one."""
        if self.post_a is not None and self.post_b is not None:
            return ((self.post_a[0] + self.post_b[0]) * 0.5,
                    (self.post_a[1] + self.post_b[1]) * 0.5)
        return self.center

    def center_norm(self, width, height):
        """Goal centre as normalized [-1, 1] image coordinates.

        Same convention as yolo_ball_detector.py: -1 = left/top edge,
        +1 = right/bottom edge. This is what a head/body aiming loop wants.
        """
        cx, cy = self.goal_center
        if width <= 0 or height <= 0:
            return (0.0, 0.0)
        return ((cx / float(width)) * 2.0 - 1.0,
                (cy / float(height)) * 2.0 - 1.0)

    def describe(self):
        cx, cy = self.goal_center
        return ('center=(%.0f,%.0f) width=%.0fpx len=%.0fpx angle=%+.1fdeg '
                'conf=%.2f posts=%d marks=%d%s'
                % (cx, cy, self.width_px, self.length_px, self.angle_deg,
                   self.confidence, self.n_posts, self.n_landmarks,
                   ' STALE(%.2fs)' % self.age_sec if self.stale else ''))


class GoalLineEstimator:
    """Turns landmark detections into a smoothed goal-line estimate.

    Algorithm, per frame:

      1. Every ``goalpost`` box contributes its FOOT — the bottom-centre of the
         box, where the post meets the ground. Those feet are the only points
         guaranteed to be on the goal line.
      2. Fit a line through the feet (total least squares).
      3. Optionally pull in L/T/X junctions lying within ``landmark_band_px`` of
         that line and refit — a junction on the goal line extends the segment
         out towards the field corner and stabilises the angle.
      4. Reject the fit if the support points are not collinear enough
         (``min_linearity``) or if the segment is shorter than
         ``min_length_px``.
      5. Exponentially smooth the endpoints against the previous frame, and when
         a frame has no usable fit, hold the previous line for ``hold_sec``
         instead of blinking out.
    """

    def __init__(self, min_goalposts=2, use_landmarks=True,
                 landmark_band_px=40.0, min_linearity=0.90,
                 min_length_px=30.0, smoothing=0.5, hold_sec=0.4):
        self.min_goalposts = max(1, int(min_goalposts))
        self.use_landmarks = bool(use_landmarks)
        self.landmark_band_px = float(landmark_band_px)
        self.min_linearity = float(min_linearity)
        self.min_length_px = float(min_length_px)
        # 0 = no smoothing (raw), 0.9 = very heavy. Applied to the endpoints.
        self.smoothing = min(0.95, max(0.0, float(smoothing)))
        self.hold_sec = max(0.0, float(hold_sec))

        self._last = None
        self._last_time = None
        self.last_reject = ''

    # -- helpers ---------------------------------------------------------
    @staticmethod
    def split_detections(detections):
        """Split ``(cx, cy, w, h, name, conf)`` tuples into posts / junctions.

        Posts come back as ``(foot_xy, conf)``; junctions as ``(center_xy,
        name, conf)``.
        """
        posts, marks = [], []
        for cx, cy, w, h, name, conf in detections:
            key = str(name).strip().lower()
            if key == GOALPOST_CLASS:
                posts.append((np.array([cx, cy + h * 0.5]), float(conf)))
            elif key in LINE_LANDMARK_CLASSES:
                marks.append((np.array([cx, cy]), key, float(conf)))
        return posts, marks

    def _smooth(self, fresh):
        """EMA the endpoints of ``fresh`` towards the previous estimate."""
        prev = self._last
        if prev is None or self.smoothing <= 0.0:
            return fresh
        a = self.smoothing

        def mix(new, old):
            if new is None or old is None:
                return new
            return (old[0] * a + new[0] * (1.0 - a),
                    old[1] * a + new[1] * (1.0 - a))

        fresh.p1 = mix(fresh.p1, prev.p1)
        fresh.p2 = mix(fresh.p2, prev.p2)
        fresh.post_a = mix(fresh.post_a, prev.post_a)
        fresh.post_b = mix(fresh.post_b, prev.post_b)
        if prev.width_px > 0.0 and fresh.width_px > 0.0:
            fresh.width_px = prev.width_px * a + fresh.width_px * (1.0 - a)
        fresh.confidence = prev.confidence * a + fresh.confidence * (1.0 - a)
        return fresh

    # -- main entry point -------------------------------------------------
    def update(self, detections, now=None):
        """Feed one frame of detections. Returns a ``GoalLine`` or ``None``."""
        now = time.monotonic() if now is None else float(now)
        fresh = self._fit(detections)

        if fresh is not None:
            fresh = self._smooth(fresh)
            self._last = fresh
            self._last_time = now
            return fresh

        # No fit this frame: hold the previous line briefly so a one-frame miss
        # (very common at 12 FPS on a walking robot) does not drop the target.
        if self._last is not None and self._last_time is not None:
            age = now - self._last_time
            if age <= self.hold_sec:
                held = GoalLine(self._last.p1, self._last.p2,
                                self._last.post_a, self._last.post_b,
                                self._last.width_px, self._last.confidence,
                                self._last.n_posts, self._last.n_landmarks,
                                self._last.linearity, stale=True, age_sec=age)
                return held
            self._last = None
            self._last_time = None
        return None

    def reset(self):
        self._last = None
        self._last_time = None

    def _fit(self, detections):
        posts, marks = self.split_detections(detections)
        self.last_reject = ''

        if len(posts) < self.min_goalposts:
            self.last_reject = ('only %d goalpost(s), need %d'
                                % (len(posts), self.min_goalposts))
            return None

        feet = [p for p, _ in posts]
        seed = _fit_line(feet) if len(feet) >= 2 else None
        if seed is None:
            # A single post cannot define a line on its own; the junctions have
            # to supply the second point.
            if not (self.use_landmarks and marks):
                self.last_reject = 'one goalpost and no usable junction'
                return None
            support = feet + [p for p, _, _ in marks]
            used_marks = len(marks)
        else:
            centroid, direction, _ = seed
            support = list(feet)
            used_marks = 0
            if self.use_landmarks:
                for point, _name, _conf in marks:
                    rel = point - centroid
                    # Perpendicular distance = |cross(rel, direction)|.
                    dist = abs(rel[0] * direction[1] - rel[1] * direction[0])
                    if dist <= self.landmark_band_px:
                        support.append(point)
                        used_marks += 1

        fit = _fit_line(support)
        if fit is None:
            self.last_reject = 'degenerate support set'
            return None
        centroid, direction, linearity = fit
        if linearity < self.min_linearity:
            self.last_reject = ('support not collinear (%.2f < %.2f)'
                                % (linearity, self.min_linearity))
            return None
        direction = _orient(direction)

        # Project every support point on the fitted line; the extremes are the
        # visible extent of the goal line.
        proj = [float(np.dot(np.asarray(p) - centroid, direction))
                for p in support]
        p1 = centroid + min(proj) * direction
        p2 = centroid + max(proj) * direction
        p1, p2 = _order_endpoints(p1, p2)

        # Post-to-post span, which is the actual GOAL width — the segment above
        # may reach further, out to a junction on the same line.
        post_a = post_b = None
        width_px = 0.0
        if len(feet) >= 2:
            post_proj = sorted(
                ((float(np.dot(np.asarray(p) - centroid, direction)), p)
                 for p in feet), key=lambda item: item[0])
            t_lo, foot_lo = post_proj[0]
            t_hi, foot_hi = post_proj[-1]
            post_a = centroid + t_lo * direction
            post_b = centroid + t_hi * direction
            post_a, post_b = _order_endpoints(post_a, post_b)
            width_px = abs(t_hi - t_lo)

        line = GoalLine(p1, p2, post_a, post_b, width_px,
                        confidence=float(np.mean([c for _, c in posts])),
                        n_posts=len(posts), n_landmarks=used_marks,
                        linearity=linearity)
        if line.length_px < self.min_length_px:
            self.last_reject = ('segment too short (%.0f < %.0f px)'
                                % (line.length_px, self.min_length_px))
            return None
        return line


# ---------------------------------------------------------------------------
# ultralytics glue
# ---------------------------------------------------------------------------
def extract_detections(results, class_names):
    """Convert ultralytics results into ``(cx, cy, w, h, name, conf)`` tuples."""
    detections = []
    if not results:
        return detections
    boxes = getattr(results[0], 'boxes', None)
    if boxes is None or boxes.xyxy is None:
        return detections

    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy() if boxes.conf is not None else None
    clses = boxes.cls.cpu().numpy() if boxes.cls is not None else None
    for idx, (x1, y1, x2, y2) in enumerate(xyxy):
        cls_id = int(clses[idx]) if clses is not None else -1
        detections.append((
            float((x1 + x2) * 0.5), float((y1 + y2) * 0.5),
            float(x2 - x1), float(y2 - y1),
            str(class_names.get(cls_id, cls_id)),
            float(confs[idx]) if confs is not None else 1.0,
        ))
    return detections


def speed_of(results):
    """Return ultralytics' (pre, infer, post) milliseconds, or (None,)*3."""
    speed = getattr(results[0], 'speed', None) if results else None
    if not speed:
        return (None, None, None)
    return (speed.get('preprocess'), speed.get('inference'),
            speed.get('postprocess'))


# ---------------------------------------------------------------------------
# drawing
# ---------------------------------------------------------------------------
def draw_goal_line(image, goal, thickness=3):
    """Overlay the goal line, the post feet and a short label."""
    import cv2

    if goal is None or image is None:
        return image
    colour = (0, 165, 255) if goal.stale else (0, 255, 255)   # BGR
    p1 = (int(round(goal.p1[0])), int(round(goal.p1[1])))
    p2 = (int(round(goal.p2[0])), int(round(goal.p2[1])))
    cv2.line(image, p1, p2, colour, thickness, cv2.LINE_AA)
    cv2.circle(image, p1, thickness + 2, colour, -1, cv2.LINE_AA)
    cv2.circle(image, p2, thickness + 2, colour, -1, cv2.LINE_AA)

    for foot in (goal.post_a, goal.post_b):
        if foot is None:
            continue
        cv2.drawMarker(image, (int(round(foot[0])), int(round(foot[1]))),
                       (255, 0, 255), cv2.MARKER_TRIANGLE_UP, 16, 2,
                       cv2.LINE_AA)

    cx, cy = goal.goal_center
    cv2.drawMarker(image, (int(round(cx)), int(round(cy))), (0, 0, 255),
                   cv2.MARKER_CROSS, 20, 2, cv2.LINE_AA)

    label = 'GOAL LINE %+.1f deg  w=%.0fpx  conf=%.2f%s' % (
        goal.angle_deg, goal.width_px, goal.confidence,
        '  (held)' if goal.stale else '')
    org = (max(4, min(p1[0], p2[0])), max(18, min(p1[1], p2[1]) - 10))
    cv2.putText(image, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                colour, 1, cv2.LINE_AA)
    return image


def draw_hud(image, lines, origin=(8, 22), scale=0.55):
    """Print a few lines of text (FPS readout) in the image corner."""
    import cv2

    if image is None:
        return image
    x, y = origin
    step = int(24 * scale / 0.55)
    for i, text in enumerate(lines):
        pos = (x, y + i * step)
        cv2.putText(image, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return image
