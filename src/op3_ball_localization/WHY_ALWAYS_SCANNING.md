# Why Is Head Always Scanning?

## The Issue
The head keeps scanning and never stops to track the ball.

## Root Cause
**The head tracking node only switches from SCANNING to TRACKING mode when it receives ball detections.**

If it's always scanning, it means:
1. ✅ YOLO detector is running
2. ✅ Bridge is running and connected
3. ✅ Head tracking node is running
4. ❌ **YOLO is NOT detecting any "ball" objects**

## Understanding the Flow

```
Camera Feed
    ↓
YOLO Detector (looks for "ball" class)
    ↓
    ├─ Detects "ball" → publishes to /vision/yolo/detections
    └─ No "ball" found → publishes empty detections
    ↓
Bridge (filters for class_id == "ball")
    ↓
    ├─ Ball found → publishes circle to /ball_detector_node/circle_set
    └─ No ball → publishes empty circles
    ↓
Head Tracking Node
    ↓
    ├─ Receives ball → TRACKING mode (locks onto ball)
    └─ No ball → SCANNING mode (searches for ball)
```

## How to Check What's Happening

### 1. Check if YOLO is detecting anything

With the improved logging, you should see messages like:

**If YOLO detects objects:**
```
[yolo_to_demo_bridge]: YOLO detected 3 objects: ['person', 'chair', 'bottle']
[yolo_to_demo_bridge]: No ball detected in this frame
```

**If YOLO detects a ball:**
```
[yolo_to_demo_bridge]: YOLO detected 1 objects: ['ball']
[yolo_to_demo_bridge]: ✓ BALL FOUND: center=(0.23, -0.15), radius=0.045, conf=0.87
```

**If YOLO detects nothing:**
```
[yolo_to_demo_bridge]: No ball detected in this frame
```

### 2. Check YOLO Configuration

The bridge is looking for `class_id == "ball"`. Check your YOLO model:

**What class names does YOLO use?**
- COCO model: "sports ball" (NOT "ball")
- Custom model: depends on your training

**Check YOLO config:**
```bash
find . -name "yolo.yaml" -type f | xargs cat
```

Look for `class_names` - does it include "ball"?

### 3. Possible Issues

**Issue 1: YOLO model uses "sports ball" instead of "ball"**

If YOLO is trained on COCO dataset, the class is called "sports ball", not "ball".

**Fix:** Update the bridge to look for the correct class name.

**Issue 2: No ball in camera view**

The head is scanning but there's literally no ball visible to the camera.

**Fix:** Place a ball in front of the robot.

**Issue 3: Ball confidence too low**

YOLO detects the ball but with low confidence, so it's filtered out.

**Fix:** Lower the confidence threshold or improve lighting/ball visibility.

## Quick Diagnostic Commands

### Check what YOLO is detecting (requires soccer_msgs):
```bash
export ROS_DOMAIN_ID=1
ros2 run op3_ball_localization yolo_to_demo_bridge
# Watch the logs - it will now show what objects are detected
```

### Monitor the head tracking node:
```bash
export ROS_DOMAIN_ID=1
# Watch for [SCANNING] vs [TRACKING] messages
# If always [SCANNING], no ball is being detected
```

## Solution Options

### Option 1: Fix Class Name (if YOLO uses "sports ball")

Edit `yolo_to_demo_bridge.py` line 48:
```python
# Change from:
if bbox.class_id == "ball" and bbox.probability > best_confidence:

# To:
if bbox.class_id == "sports ball" and bbox.probability > best_confidence:
```

### Option 2: Accept Multiple Class Names

Make it more flexible:
```python
# Accept multiple possible ball class names
ball_classes = ["ball", "sports ball", "soccer ball", "football"]
if bbox.class_id in ball_classes and bbox.probability > best_confidence:
```

### Option 3: Check YOLO Model

Verify what your YOLO model is trained to detect. You might need to:
- Retrain YOLO with "ball" as a class
- Use a different model
- Adjust the class name in the bridge

## After Rebuild

```bash
colcon build --packages-select op3_ball_localization
source install/setup.bash

# Restart the launch file
export ROS_DOMAIN_ID=1
ros2 launch op3_ball_localization yolo_scan_only.launch.py
```

Now watch the logs carefully:
- What objects is YOLO detecting?
- Is "ball" in the list?
- If yes → head should stop scanning and start tracking
- If no → that's why it keeps scanning

## Expected Behavior

**When no ball visible:**
```
[yolo_to_demo_bridge]: No ball detected in this frame
[head_tracking_node]: [SCANNING] pan=0.15, tilt=-0.50, dir=+
```

**When ball detected:**
```
[yolo_to_demo_bridge]: ✓ BALL FOUND: center=(0.23, -0.15), radius=0.045, conf=0.87
[head_tracking_node]: [TRACKING] pan=0.25, tilt=-0.40, ball_error=(0.12, -0.05)
```

## Summary

The head is **correctly** always scanning because:
- ✅ The system is working as designed
- ✅ Scanning is the default mode when no ball is detected
- ❌ YOLO is not detecting "ball" objects

**Next step:** Check the bridge logs to see what YOLO is actually detecting, then fix the class name mismatch or add a ball to the camera view.
