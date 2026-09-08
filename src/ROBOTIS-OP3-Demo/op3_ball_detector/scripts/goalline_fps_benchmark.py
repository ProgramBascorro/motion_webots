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

"""Standalone goal-line detector + FPS report — no ROS needed.

Same detection code as yolo_goalline_detector.py (both import goalline_core),
but reading from a video file, a camera or a folder of images, and printing the
frame rate it achieved. Use it to answer "how many FPS does this model give me
on this machine?" before wiring it into the robot.

Every run ends with a summary block: average FPS, min/max, the pre/infer/post
millisecond breakdown, and how often a goal line was actually found.

Examples
--------
    # a recorded clip, headless
    ./goalline_fps_benchmark.py --source recording_20260128_044704.mp4

    # the robot's camera, 300 frames, save an annotated video
    ./goalline_fps_benchmark.py --source 0 --max-frames 300 --save out.mp4

    # compare backends (the .pt / .onnx weights ship in the same folder)
    ./goalline_fps_benchmark.py --source clip.mp4 \
        --model .../model/best_landmark_v8n.pt

Inside the project container:

    docker exec -it op3 bash -lc \
      "ros2 run op3_ball_detector goalline_fps_benchmark.py --source clip.mp4"
"""

import argparse
import os
import signal
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detector_fps import FpsMeter  # noqa: E402
from goalline_core import (  # noqa: E402
    GoalLineEstimator,
    draw_goal_line,
    draw_hud,
    extract_detections,
    speed_of,
)

IMAGE_SUFFIXES = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')
DEFAULT_MODEL = 'model/best_landmark_v8n_openvino_model'


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        description='Goal-line (garis gawang) detection + FPS report.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    p.add_argument('--source', default='0',
                   help='Video file, camera index ("0"), or a folder of images')
    p.add_argument('--model', default=DEFAULT_MODEL,
                   help='OpenVINO model DIR, or a .pt / .onnx file. A relative '
                        'path is looked up in the package share dir and in the '
                        'source tree')
    p.add_argument('--task', default='detect',
                   help='Ultralytics task (OpenVINO/ONNX carry no reliable task)')
    p.add_argument('--imgsz', type=int, default=640,
                   help='MUST match the exported size (best_landmark_v8n = 640)')
    p.add_argument('--conf', type=float, default=0.5, help='Confidence threshold')
    p.add_argument('--iou', type=float, default=0.45, help='NMS IoU threshold')
    p.add_argument('--device', default='cpu', help='cpu / 0 / cuda:0 ...')
    p.add_argument('--max-det', type=int, default=30, help='Max boxes per frame')

    p.add_argument('--max-frames', type=int, default=0,
                   help='Stop after N frames (0 = until the source ends)')
    p.add_argument('--max-sec', type=float, default=0.0,
                   help='Stop after N seconds of processing (0 = no limit)')
    p.add_argument('--warmup', type=int, default=3,
                   help='Warm-up inferences excluded from the FPS numbers')

    p.add_argument('--min-goalposts', type=int, default=2,
                   help='Posts needed before a line is fitted (2 = both posts)')
    p.add_argument('--no-landmarks', action='store_true',
                   help='Fit on goalpost feet only, ignore L/T/X junctions')
    p.add_argument('--band', type=float, default=40.0,
                   help='How close (px) a junction must be to count as on-line')
    p.add_argument('--linearity', type=float, default=0.90,
                   help='Reject fits below this collinearity (0..1)')
    p.add_argument('--min-length', type=float, default=30.0,
                   help='Reject a goal-line segment shorter than this (px)')
    p.add_argument('--smoothing', type=float, default=0.5,
                   help='Endpoint EMA weight of the previous frame (0 = raw)')
    p.add_argument('--hold', type=float, default=0.4,
                   help='Keep the last line this long (s) when a frame misses')

    p.add_argument('--save', default='',
                   help='Write an annotated video here (e.g. out.mp4)')
    p.add_argument('--show', action='store_true',
                   help='Open a preview window (needs a display)')
    p.add_argument('--no-boxes', action='store_true',
                   help='Draw only the goal line, not the raw YOLO boxes')
    p.add_argument('--csv', default='',
                   help='Write a per-frame CSV (frame,ms,fps,found,...)')
    p.add_argument('--log-period', type=float, default=2.0,
                   help='Seconds between live FPS lines')
    p.add_argument('--quiet', action='store_true',
                   help='Only print the final summary')
    return p


def resolve_model(path):
    """Find the weights whether we run from ROS, the source tree, or anywhere."""
    if os.path.isabs(path) or os.path.exists(path):
        return path
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('op3_ball_detector'), path))
    except Exception:  # noqa: BLE001 - fine, we are probably outside ROS
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(os.path.dirname(here), path))  # ../<path>
    candidates.append(os.path.join(here, path))
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    raise FileNotFoundError(
        'Model not found: %s (looked in %s)' % (path, ', '.join(candidates)))


# ---------------------------------------------------------------------------
# frame sources
# ---------------------------------------------------------------------------
class FrameSource:
    """Yields BGR frames from a camera, a video file or a folder of images."""

    def __init__(self, source):
        self.kind = 'video'
        self.name = str(source)
        self.source_fps = 0.0
        self.total = 0
        self._files = []
        self._index = 0

        if os.path.isdir(source):
            self.kind = 'images'
            self._files = sorted(
                os.path.join(source, f) for f in os.listdir(source)
                if f.lower().endswith(IMAGE_SUFFIXES))
            if not self._files:
                raise ValueError('No images in %s' % source)
            self.total = len(self._files)
            return

        target = int(source) if str(source).isdigit() else source
        if isinstance(target, str) and not os.path.exists(target):
            raise FileNotFoundError('Source not found: %s' % target)
        self.cap = cv2.VideoCapture(target)
        if not self.cap.isOpened():
            raise RuntimeError('Cannot open source: %s' % source)
        if isinstance(target, int):
            self.kind = 'camera'
        self.source_fps = float(self.cap.get(cv2.CAP_PROP_FPS) or 0.0)
        self.total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)

    def read(self):
        if self.kind == 'images':
            if self._index >= len(self._files):
                return None
            frame = cv2.imread(self._files[self._index])
            self._index += 1
            return frame
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self):
        if self.kind != 'images':
            self.cap.release()


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(argv=None):
    args = build_parser().parse_args(argv)

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        print('Failed to import ultralytics (pip3 install ultralytics): %s'
              % exc, file=sys.stderr)
        return 2

    model_path = resolve_model(args.model)
    print('model  : %s' % model_path)
    model = YOLO(model_path, task=args.task) if args.task else YOLO(model_path)
    class_names = dict(model.names)
    print('classes: %s' % class_names)

    source = FrameSource(args.source)
    print('source : %s (%s%s%s)'
          % (source.name, source.kind,
             ', %d frames' % source.total if source.total else '',
             ', %.1f fps' % source.source_fps if source.source_fps else ''))

    predict_kwargs = dict(conf=args.conf, iou=args.iou, imgsz=args.imgsz,
                          device=args.device, max_det=args.max_det,
                          verbose=False)

    # Warm-up. The VERY first inference pays for lazy graph/IR compilation (on
    # OpenVINO that is ~200 ms against ~20 ms steady state), so it is thrown
    # away rather than timed — otherwise the reported ceiling is nonsense.
    if args.warmup > 0:
        import numpy as np
        blank = np.zeros((args.imgsz, args.imgsz, 3), dtype='uint8')
        model.predict(blank, **predict_kwargs)
        started = time.monotonic()
        for _ in range(args.warmup):
            model.predict(blank, **predict_kwargs)
        warm = time.monotonic() - started
        print('warm-up: %d frames, %.1f ms/frame (ceiling ~%.1f FPS)'
              % (args.warmup, warm / args.warmup * 1000.0, args.warmup / warm))

    estimator = GoalLineEstimator(
        min_goalposts=args.min_goalposts,
        use_landmarks=not args.no_landmarks,
        landmark_band_px=args.band,
        min_linearity=args.linearity,
        min_length_px=args.min_length,
        smoothing=args.smoothing,
        hold_sec=args.hold)
    fps = FpsMeter(window_sec=2.0, name=os.path.basename(str(args.source)),
                   hit_label='goal line found')

    writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, 'w')
        csv_file.write('frame,loop_ms,infer_ms,fps_window,found,posts,'
                       'marks,center_x,center_y,width_px,angle_deg,conf\n')

    last_log = time.monotonic()
    frame_no = 0
    run_started = time.monotonic()
    interrupted = False

    try:
        while True:
            frame = source.read()
            if frame is None:
                break
            frame_no += 1
            loop_start = time.monotonic()

            results = model.predict(frame, **predict_kwargs)
            detections = extract_detections(results, class_names)
            goal = estimator.update(detections, now=loop_start)

            pre_ms, infer_ms, post_ms = speed_of(results)
            loop_ms = (time.monotonic() - loop_start) * 1000.0
            fps_now = fps.tick(now=loop_start, infer_ms=infer_ms, pre_ms=pre_ms,
                               post_ms=post_ms, loop_ms=loop_ms,
                               hit=goal is not None)

            if csv_file:
                cx, cy = goal.goal_center if goal else (0.0, 0.0)
                csv_file.write(
                    '%d,%.2f,%.2f,%.2f,%d,%d,%d,%.1f,%.1f,%.1f,%.2f,%.3f\n'
                    % (frame_no, loop_ms, infer_ms or 0.0, fps_now,
                       1 if goal else 0,
                       goal.n_posts if goal else 0,
                       goal.n_landmarks if goal else 0,
                       cx, cy,
                       goal.width_px if goal else 0.0,
                       goal.angle_deg if goal else 0.0,
                       goal.confidence if goal else 0.0))

            if args.save or args.show:
                try:
                    annotated = (results[0].plot()
                                 if (results and not args.no_boxes)
                                 else frame.copy())
                except Exception:  # noqa: BLE001
                    annotated = frame.copy()
                draw_goal_line(annotated, goal)
                draw_hud(annotated, ['FPS %.1f (avg %.1f)'
                                     % (fps_now, fps.average_fps),
                                     goal.describe() if goal else 'no goal line'])
                if args.save and writer is None:
                    height, width = annotated.shape[:2]
                    out_fps = source.source_fps if source.source_fps > 0 else 15.0
                    writer = cv2.VideoWriter(
                        args.save, cv2.VideoWriter_fourcc(*'mp4v'),
                        out_fps, (width, height))
                if writer is not None:
                    writer.write(annotated)
                if args.show:
                    cv2.imshow('goal line', annotated)
                    if (cv2.waitKey(1) & 0xFF) in (27, ord('q')):
                        break

            now = time.monotonic()
            if not args.quiet and (now - last_log) >= max(0.2, args.log_period):
                last_log = now
                state = goal.describe() if goal else (
                    'no goal line (%s)' % (estimator.last_reject or 'nothing seen'))
                print('[%5d] %s | %s' % (frame_no, fps.live_line(), state))

            if args.max_frames and frame_no >= args.max_frames:
                break
            if args.max_sec and (now - run_started) >= args.max_sec:
                break
    except KeyboardInterrupt:
        interrupted = True
    finally:
        source.release()
        if writer is not None:
            writer.release()
        if csv_file:
            csv_file.close()
        if args.show:
            cv2.destroyAllWindows()

    # A second Ctrl-C must not abort the report the user is waiting for; a
    # third one still kills the process the ordinary way.
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except ValueError:
        pass

    print('')
    if interrupted:
        print('(interrupted)')
    for line in fps.summary_lines(
            title='FPS SUMMARY - goal line @ %s' % os.path.basename(model_path)):
        print(line)
    if args.save and writer is not None:
        print('annotated video : %s' % args.save)
    if args.csv:
        print('per-frame csv   : %s' % args.csv)
    return 0 if fps.frame_count else 1


if __name__ == '__main__':
    sys.exit(main())
