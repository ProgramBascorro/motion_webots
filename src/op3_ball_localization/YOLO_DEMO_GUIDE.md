# YOLO-Based OP3 Demo Guide

## Overview

This setup uses the **proven OP3 demo ball-following behavior** but replaces the HSV color detector with **YOLO object detection** for better reliability.

## Architecture

```
YOLO Detector → Bridge → OP3 Demo → Robot Motion
    ↓               ↓          ↓
 BoundingBoxes  CircleSet   Tracking
                            Following
                            Kicking
```

### Components:

1. **YOLO Detector** (`op3_yolo_vision`)
   - Detects ball using neural network
   - Publishes: `/vision/yolo/detections` (BoundingBoxes)

2. **YOLO-to-Demo Bridge** (NEW!)
   - Converts YOLO format → CircleSetStamped
   - Publishes: `/ball_detector_node/circle_set`
   - Acts as drop-in replacement for HSV detector

3. **OP3 Demo** (`op3_demo`)
   - Ball tracking (head movement)
   - Ball following (walking)
   - Kick execution
   - **Unchanged** - uses existing proven code!

## Why This is Better

### vs Simple Ball Tracker:
- ✅ **Proven motion code** - OP3 demo is battle-tested
- ✅ **No buggy walking** - uses ROBOTIS walking params
- ✅ **Kick integration** - automatic kicking when close
- ✅ **Better head tracking** - smoother PID control

### vs HSV Ball Detector:
- ✅ **More reliable** - YOLO works in varying lighting
- ✅ **No color tuning** - works out of the box
- ✅ **Handles occlusion** - better at detecting partial balls

## Quick Start

### Method 1: Single Launch Command
```bash
source install/setup.bash
ros2 launch op3_ball_localization yolo_demo.launch.py
```

This launches everything:
- OP3 Manager (robot control)
- YOLO Vision
- YOLO-to-Demo Bridge
- OP3 Demo Node

### Method 2: Manual Control (for debugging)

**Terminal 1: YOLO**
```bash
ros2 launch op3_yolo_vision yolo.launch.py
```

**Terminal 2: Bridge**
```bash
ros2 run op3_ball_localization yolo_to_demo_bridge
```

**Terminal 3: OP3 Demo**
```bash
ros2 launch op3_demo demo.launch.xml with_manager:=true
```

## How the Bridge Works

### Input (from YOLO):
```python
BoundingBox:
  xmin: 100, ymin: 150
  xmax: 200, ymax: 250
  class_id: "ball"
  probability: 0.85
```

### Output (to OP3 Demo):
```python
Circle2D:
  center.x: 0.2    # Normalized [-1, 1]
  center.y: 0.3    # Normalized [-1, 1]
  z: 0.15          # Size indicator
```

The demo expects:
- `x = -1`: ball on left edge
- `x = 0`: ball centered horizontally
- `x = +1`: ball on right edge
- `y = -1`: ball at top
- `y = +1`: ball at bottom
- `z`: ball size (larger = closer)

## Monitoring

```bash
# Check YOLO detections
ros2 topic echo /vision/yolo/detections

# Check bridge output (what demo sees)
ros2 topic echo /ball_detector_node/circle_set

# Check demo commands
ros2 topic echo /robotis/walking/command

# Monitor YOLO performance
ros2 topic hz /vision/yolo/detections
```

## Tuning

### YOLO Performance
Edit `src/op3_yolo_vision/config/yolo.yaml`:
```yaml
input_size: 640              # Reduce to 416 for speed
publish_debug: false         # Disable for performance
frame_skip: 1                # Increase to skip frames
ball_confidence_threshold: 0.2  # Adjust sensitivity
```

### Demo Behavior
Edit demo node parameters (in launch file):
```python
parameters=[{
    'grass_demo': False,     # Set True for outdoor
    'p_gain': 0.45,          # Head tracking proportional gain
    'd_gain': 0.045          # Head tracking derivative gain
}]
```

### Walking Speed
The demo uses its own walking parameters. To adjust, you need to modify the demo source code or use dynamic reconfigure.

## Troubleshooting

### "No ball detected"
- Check YOLO is running: `ros2 topic hz /vision/yolo/detections`
- Check confidence threshold in YOLO config
- Verify ball is in camera view

### "Robot not moving"
- Check bridge is running: `ros2 topic hz /ball_detector_node/circle_set`
- Verify demo received message: `ros2 topic echo /ball_detector_node/circle_set`
- Check walking module is enabled

### "Head not tracking"
- Ensure head_control_module is enabled
- Check joint states: `ros2 topic echo /robotis/present_joint_states`

### "Bridge not publishing"
- Check YOLO detections have "ball" class_id
- Verify bbox coordinates are valid
- Check terminal for bridge errors

## Comparison to Other Approaches

| Feature | YOLO Demo | Simple Tracker | Ball Localizer |
|---------|-----------|----------------|----------------|
| **Motion Quality** | ⭐⭐⭐⭐⭐ Proven | ⭐⭐⭐ Basic | ⭐⭐⭐⭐ Good |
| **Setup Complexity** | ⭐⭐⭐ Medium | ⭐⭐ Easy | ⭐⭐⭐⭐ Complex |
| **Detection** | ⭐⭐⭐⭐ YOLO | ⭐⭐⭐⭐ YOLO | ⭐⭐⭐⭐ YOLO |
| **Kick Support** | ✅ Yes | ❌ No | ✅ Yes |
| **3D Positioning** | ❌ No | ❌ No | ✅ Yes |
| **Performance** | ⭐⭐⭐⭐ Fast | ⭐⭐⭐⭐⭐ Fastest | ⭐⭐⭐ Slower |

## Recommended Use

✅ **Use YOLO Demo when:**
- You want reliable ball following
- You need kick functionality
- You trust ROBOTIS code over custom
- You don't need 3D localization

❌ **Use other approaches when:**
- You need precise 3D ball position
- You want custom walking behavior
- You're doing research/experimentation

## Next Steps

1. **Test with real robot**
2. **Tune YOLO confidence if needed**
3. **Adjust demo PID gains for your robot**
4. **Test kicking behavior**

Happy robot soccer! ⚽🤖
