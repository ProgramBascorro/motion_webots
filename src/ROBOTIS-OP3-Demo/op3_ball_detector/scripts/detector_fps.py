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

"""One frame-rate meter, shared by every detector in this package.

Ultralytics reports per-stage MILLISECONDS in ``results[].speed``, which answers
"how expensive is one frame" but not "how many frames per second am I actually
getting" — the two differ whenever the loop waits on the camera, drops frames,
or is throttled by ``detection_rate``. This class measures the second one, from
real wall-clock arrivals, and keeps the millisecond breakdown alongside it for
diagnosis.

Imported by:

    yolo_ball_detector.py       ball -> CircleSetStamped
    yolo_landmark_detector.py   5-class landmark -> Detection2DArray
    goalline_core.py            geometry for the goal-line detector
    goalline_fps_benchmark.py   offline FPS benchmark

All of those are installed into the SAME directory (lib/op3_ball_detector), so
each imports this after putting its own directory on sys.path. Nothing here
imports rclpy — keep it that way so the offline benchmark stays ROS-free.
"""

import collections
import time

import numpy as np


class FpsMeter:
    """Sliding-window frame-rate meter plus an end-of-run summary.

    ``window_fps`` is what you want on screen — it reacts within a second or
    two — while ``average_fps`` is the honest "what did this run actually
    achieve" number reported when the program stops.

    Every duration is kept in milliseconds, matching ``results[0].speed``.

    ``hit_label`` names the per-frame event the caller counts through
    ``tick(hit=...)``: "ball seen", "goal line found", "frame with a
    landmark". Pass None to leave that line out of the summary.
    """

    def __init__(self, window_sec=2.0, name='detector', keep_samples=20000,
                 hit_label='hits'):
        self.name = name
        self.window_sec = max(0.2, float(window_sec))
        self.keep_samples = int(keep_samples)
        self.hit_label = hit_label

        self._stamps = collections.deque()   # frame times inside the window
        self._t_first = None
        self._t_last = None
        self._count = 0
        self._hits = 0

        self._infer_ms = []
        self._pre_ms = []
        self._post_ms = []
        self._loop_ms = []

        self._fps_min = None
        self._fps_max = None

    # -- recording -------------------------------------------------------
    def tick(self, now=None, infer_ms=None, pre_ms=None, post_ms=None,
             loop_ms=None, hit=None):
        """Record one processed frame. Returns the current window FPS."""
        now = time.monotonic() if now is None else float(now)

        if self._t_first is None:
            self._t_first = now
        self._t_last = now
        self._count += 1
        if hit:
            self._hits += 1

        self._stamps.append(now)
        while self._stamps and (now - self._stamps[0]) > self.window_sec:
            self._stamps.popleft()

        for value, bucket in ((infer_ms, self._infer_ms),
                              (pre_ms, self._pre_ms),
                              (post_ms, self._post_ms),
                              (loop_ms, self._loop_ms)):
            if value is None:
                continue
            if len(bucket) < self.keep_samples:
                bucket.append(float(value))

        fps = self.window_fps
        # Only let the window set min/max once it is actually full, otherwise
        # the first two frames after start-up would peg "min FPS" at ~0.
        span = self._stamps[-1] - self._stamps[0] if len(self._stamps) > 1 else 0.0
        if fps > 0.0 and span >= self.window_sec * 0.8:
            self._fps_min = fps if self._fps_min is None else min(self._fps_min, fps)
            self._fps_max = fps if self._fps_max is None else max(self._fps_max, fps)
        return fps

    # -- readings --------------------------------------------------------
    @property
    def window_fps(self):
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 0.0 else 0.0

    @property
    def average_fps(self):
        """Frames per second over the whole run (n-1 intervals / elapsed)."""
        if self._count < 2 or self._t_first is None:
            return 0.0
        elapsed = self._t_last - self._t_first
        return (self._count - 1) / elapsed if elapsed > 0.0 else 0.0

    @property
    def elapsed_sec(self):
        if self._t_first is None:
            return 0.0
        return self._t_last - self._t_first

    @property
    def frame_count(self):
        return self._count

    @property
    def hit_count(self):
        return self._hits

    @property
    def last_infer_ms(self):
        return self._infer_ms[-1] if self._infer_ms else None

    @staticmethod
    def _stat(values):
        if not values:
            return None
        arr = np.asarray(values, dtype=np.float64)
        return {
            'avg': float(arr.mean()),
            'p50': float(np.percentile(arr, 50)),
            'p95': float(np.percentile(arr, 95)),
            'max': float(arr.max()),
        }

    # -- reporting -------------------------------------------------------
    def live_line(self, extra=''):
        """One-line progress log: the number the user asked to be told.

        FPS first and in FPS — the millisecond figures ride along as the
        explanation for it, never as the headline.
        """
        parts = ['FPS %5.1f (avg %.1f)' % (self.window_fps, self.average_fps)]
        if self._infer_ms:
            parts.append('infer %.1f ms' % self._infer_ms[-1])
        if self._loop_ms:
            parts.append('loop %.1f ms' % self._loop_ms[-1])
        parts.append('frames %d' % self._count)
        if extra:
            parts.append(extra)
        return ' | '.join(parts)

    def summary_lines(self, title=None, target_fps=None, extra_lines=None):
        """Multi-line end-of-run report, ready to print or log line by line."""
        title = title or ('FPS SUMMARY (%s)' % self.name)
        lines = ['===== %s =====' % title]
        lines.append('frames processed : %d' % self._count)
        lines.append('elapsed          : %.2f s' % self.elapsed_sec)
        lines.append('AVERAGE FPS      : %.2f' % self.average_fps)
        if self._fps_min is not None:
            lines.append('min / max FPS    : %.2f / %.2f'
                         % (self._fps_min, self._fps_max))
        if target_fps:
            achieved = (self.average_fps / target_fps * 100.0) if target_fps else 0.0
            lines.append('target rate      : %.1f Hz  (achieved %.0f%%)'
                         % (target_fps, achieved))

        for label, values in (('inference', self._infer_ms),
                              ('preprocess', self._pre_ms),
                              ('postprocess', self._post_ms),
                              ('loop total', self._loop_ms)):
            stat = self._stat(values)
            if stat:
                lines.append('%-16s : avg %6.1f ms  p50 %6.1f  p95 %6.1f  max %6.1f'
                             % (label, stat['avg'], stat['p50'],
                                stat['p95'], stat['max']))

        if self._count and self.hit_label:
            lines.append('%-16s : %d / %d frames (%.1f%%)'
                         % (self.hit_label, self._hits, self._count,
                            100.0 * self._hits / self._count))
        for line in (extra_lines or []):
            lines.append(line)
        lines.append('=' * len(lines[0]))
        return lines
