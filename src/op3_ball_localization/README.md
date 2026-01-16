# op3_ball_localization

Ball localization and approach behavior for OP3 using `/vision/yolo/balls`.

## Overview

The node scans the head (down, sweep left->right; up, sweep right->left) until the ball is detected.
If the ball is still missing after `scan_timeout`, it rotates the body while continuing the scan.
When the ball is detected, it tracks the ball with head pan and walks toward it.
At `pre_kick_distance`, it tilts the head down and slows for kick positioning.
At `kick_distance`, it stops walking and triggers the configured kick action.

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

- `arrive_distance`: stop distance from the ball when kicks are disabled.
- `pre_kick_distance`, `kick_distance`: distances for kick positioning and trigger.
- `kick_left_page`, `kick_right_page`: action pages for left/right kicks.
- `approach_max_x`, `approach_max_yaw`: forward/yaw limits while approaching.
- `scan_pan_min`, `scan_pan_max`, `scan_tilt_down`, `scan_tilt_up`: scan pattern.
- `scan_timeout`: time before starting body rotation.
- `search_yaw`: in-place rotation rate while searching.
- `head_track_pan_gain`: head pan tracking gain.
- `head_track_tilt_mode`: `geometry` (default), `distance`, or `fixed`.
- `camera_height`, `camera_forward_offset`, `head_track_tilt_offset`: geometry inputs for tilt.
- `head_track_tilt_far`, `head_track_tilt_near`: tilt when far vs near the ball (distance mode).
- `head_track_tilt_far_distance`, `head_track_tilt_near_distance`: distance range for tilt interpolation.
- `head_track_tilt_max_step`: limit per-update tilt change.
- `pre_kick_tilt`: head tilt used during kick positioning.
- `status_log_enabled`, `status_log_period_sec`: periodic status log for tuning distances.
