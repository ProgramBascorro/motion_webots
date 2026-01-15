# op3_ball_localization

Ball localization and approach behavior for OP3 using `/vision/yolo/balls`.

## Overview

The node scans the head (down, sweep left->right; up, sweep right->left) until the ball is detected.
If the ball is still missing after `scan_timeout`, it rotates the body while continuing the scan.
When the ball is detected, it tracks the ball with head pan and walks toward it until `arrive_distance`.

## Run

```bash
ros2 launch op3_yolo_vision yolo.launch.py
ros2 launch op3_ball_localization ball_localizer.launch.py
```

Ensure YOLO outputs points in `base`:

```yaml
# src/op3_yolo_vision/config/yolo.yaml
output_frame: base
```

## Key parameters

- `arrive_distance`: stop distance from the ball.
- `approach_max_x`, `approach_max_yaw`: forward/yaw limits while approaching.
- `scan_pan_min`, `scan_pan_max`, `scan_tilt_down`, `scan_tilt_up`: scan pattern.
- `scan_timeout`: time before starting body rotation.
- `search_yaw`: in-place rotation rate while searching.
- `head_track_pan_gain`, `head_track_tilt`: head tracking while approaching/arrived.
