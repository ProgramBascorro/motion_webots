# YOLO Demo Launch Configurations

This package provides multiple launch file configurations for the YOLO-based OP3 ball tracking system, allowing you to choose different behavior modes.

## Available Launch Files

### 1. Full Demo Mode (`yolo_full_demo.launch.py`)
**Complete soccer behavior: scan, track, walk, and kick**

```bash
ros2 launch op3_ball_localization yolo_full_demo.launch.py
```

**Behavior:**
- Head scans to search for the ball
- Tracks ball by keeping it centered in camera view
- Walks toward the ball
- Kicks when close enough

**Components:**
- YOLO detector
- YOLO-to-demo bridge
- OP3 demo node (full behavior)

---

### 2. Scan Only Mode (`yolo_scan_only.launch.py`)
**Head tracking only - no walking or kicking**

```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py
```

**Behavior:**
- Head scans to search for the ball
- Tracks ball by keeping it centered in camera view
- Does NOT walk
- Does NOT kick

**Components:**
- YOLO detector
- YOLO-to-demo bridge
- Head tracking node (standalone)

**Use Cases:**
- Testing ball detection without robot movement
- Safe testing on a table or stand
- Debugging camera/detection issues
- Demonstrating vision system only

---

### 3. Original Demo (`yolo_demo.launch.py`)
**Legacy launch file - same as full demo**

```bash
ros2 launch op3_ball_localization yolo_demo.launch.py
```

This is kept for backward compatibility and is identical to `yolo_full_demo.launch.py`.

---

## Prerequisites

All launch files require the OP3 manager and camera to be running first:

```bash
# Terminal 1: Launch manager + camera
cd /path/to/workspace
./script.sh
# Select: "op3_manager + usb_cam"
```

Wait for the manager to initialize, then launch your chosen demo in a new terminal:

```bash
# Terminal 2: Launch demo
source install/setup.bash
ros2 launch op3_ball_localization <launch_file>
```

---

## Configuration Files

### Head Tracking Configuration (`config/head_tracking.yaml`)
Used by scan-only mode to configure head tracking behavior:

```yaml
head_tracking_node:
  ros__parameters:
    # PID Control
    p_gain: 0.45
    d_gain: 0.045

    # Scanning
    scan_enabled: true
    scan_pan_min: -1.2
    scan_pan_max: 1.2
    scan_tilt_down: -0.6
    scan_tilt_up: -0.25
    scan_step: 0.2
    scan_dwell_sec: 0.8

    # Detection
    lost_timeout: 1.5
    min_ball_radius: 0.01
```

You can override parameters at launch time:

```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py \
  head_tracking_node.p_gain:=0.5 \
  head_tracking_node.scan_step:=0.3
```

---

## Quick Testing Guide

### Test Scan Only Mode

1. Launch manager + camera:
   ```bash
   ./script.sh  # Choose "op3_manager + usb_cam"
   ```

2. In another terminal, launch scan only:
   ```bash
   source install/setup.bash
   ros2 launch op3_ball_localization yolo_scan_only.launch.py
   ```

3. Verify behavior:
   - Head should scan left-right, up-down
   - When ball is detected, head should track it
   - Robot should NOT walk

4. Monitor topics:
   ```bash
   # Check ball detections
   ros2 topic echo /ball_detector_node/circle_set

   # Check head commands
   ros2 topic echo /robotis/head_control/set_joint_states

   # Verify NO walking commands (should show 0 Hz)
   ros2 topic hz /robotis/walking/command
   ```

### Test Full Demo Mode

1. Launch manager + camera (if not already running)

2. Launch full demo:
   ```bash
   source install/setup.bash
   ros2 launch op3_ball_localization yolo_full_demo.launch.py
   ```

3. Verify behavior:
   - Head should scan and track ball
   - Robot should walk toward ball
   - Robot should kick when close

4. Monitor topics:
   ```bash
   # Check all systems
   ros2 topic hz /robotis/walking/command  # Should show activity
   ros2 topic echo /ball_detector_node/circle_set
   ros2 node list | grep demo
   ```

---

## Troubleshooting

### Robot doesn't move (scan only mode)
This is expected! Scan only mode does not enable walking. Use full demo mode if you want walking behavior.

### Head doesn't track ball (scan only mode)
Check:
- Ball is being detected: `ros2 topic echo /ball_detector_node/circle_set`
- YOLO is running: `ros2 node list | grep yolo`
- Bridge is running: `ros2 node list | grep bridge`
- Head tracking node is running: `ros2 node list | grep head_tracking`

### Robot doesn't walk (full demo mode)
Check:
- Demo node is running: `ros2 node list | grep demo`
- Walking module is enabled: Check manager output
- Ball is close enough to trigger walking

---

## Architecture

### Scan Only Mode
```
Camera → YOLO Detector → YOLO-to-Demo Bridge → Head Tracking Node → Head Control
```

### Full Demo Mode
```
Camera → YOLO Detector → YOLO-to-Demo Bridge → OP3 Demo Node → Head Control + Walking + Kicking
```

---

## Files Created

**New Launch Files:**
- `launch/yolo_full_demo.launch.py` - Full soccer demo
- `launch/yolo_scan_only.launch.py` - Head tracking only

**New Nodes:**
- `op3_ball_localization/head_tracking_node.py` - Standalone head tracker

**New Config:**
- `config/head_tracking.yaml` - Head tracking parameters

**Modified:**
- `setup.py` - Added new entry points and data files

---

## Development Notes

The head tracking node (`head_tracking_node.py`) implements:
- PID control for smooth ball tracking
- Scanning pattern when ball is lost
- Normalized coordinate handling from ball detections
- Configurable parameters for tuning

It subscribes to:
- `/ball_detector_node/circle_set` (CircleSetStamped) - Ball detections

It publishes to:
- `/robotis/head_control/set_joint_states` (JointState) - Head commands
- `/robotis/enable_ctrl_module` (String) - Module enable

The node operates independently and does NOT control walking or kicking, making it safe for testing on a stand or table.
