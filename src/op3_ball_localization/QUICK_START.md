# Quick Start: YOLO + OP3 Tracking

## Two-Step Launch

### Step 1: Launch Manager + Camera (via script.sh)

```bash
./script.sh
# Select "Launch picker"
# Choose one of these:
#   - "op3_manager + usb_cam"
#   - "op3_manager.launch.py"
```

This starts:
- OP3 Manager (robot control)
- USB Camera node
- Basic robot services

**Keep this running!**

---

### Step 2A: Launch YOLO + Head-Only Tracking

In a **new terminal**:

```bash
source install/setup.bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py
```

Or via script.sh:
```bash
./script.sh
# Select "Launch picker"
# Choose "yolo_scan_only.launch.py"
```

This starts:
- YOLO detector
- YOLO-to-Demo bridge
- Head tracking node

The robot will:
- scan for the ball using the head only
- keep the ball centered with pan/tilt tracking
- not walk
- not kick

### Step 2B: Launch Full Demo

In a **new terminal**:

```bash
source install/setup.bash
ros2 launch op3_ball_localization yolo_demo.launch.py
```

Or via script.sh:
```bash
./script.sh
# Select "Launch picker"
# Choose "yolo_demo.launch.py"
```

This starts:
- YOLO detector
- YOLO-to-Demo bridge
- OP3 Demo node (ball following)

---

## What Runs Where

### Terminal 1: Manager + Camera
```
✓ op3_manager
✓ usb_cam_node
✓ /robotis/* topics
✓ /usb_cam/image_raw
```

### Terminal 2: YOLO + Head-Only Tracking
```
✓ yolo_detector
✓ yolo_to_demo_bridge
✓ head_tracking_node
```

### Terminal 2: YOLO + Demo
```
✓ yolo_detector
✓ yolo_to_demo_bridge
✓ op_demo_node
```

---

## Usage Flow

### Head-Only Test

1. **Launch manager + camera** (Terminal 1)
2. Wait for "Manager initialized"
3. **Launch YOLO + head-only tracking** (Terminal 2)
4. Place ball in front of robot
5. Robot will:
   - scan with the head when the ball is missing
   - track the ball with the head when it appears
   - keep legs/body stationary

### Full Demo

1. **Launch manager + camera** (Terminal 1)
2. Wait for "Manager initialized"
3. **Launch YOLO + demo** (Terminal 2)
4. Place ball in front of robot
5. Robot will:
   - track ball with head
   - walk toward ball
   - kick when close

---

## Quick Checks

```bash
# Is manager running?
ros2 topic list | grep robotis

# Is camera running?
ros2 topic echo /usb_cam/image_raw --once

# Is YOLO detecting?
ros2 topic hz /vision/yolo/detections

# Is bridge working?
ros2 topic echo /ball_detector_node/circle_set

# Is head tracking publishing commands?
ros2 topic echo /robotis/head_control/set_joint_states

# Is demo running?
ros2 node list | grep demo
```

---

## Stop Everything

**Terminal 2:** `Ctrl+C` (stops YOLO + demo)
**Terminal 1:** `Ctrl+C` (stops manager + camera)

Or:
```bash
./script.sh
# Choose "Exit tmux session"
```

---

## Why Two Steps?

- **Manager/Camera** = Hardware layer (runs once, stable)
- **YOLO/Demo** = Behavior layer (restart often during development)

This way you can:
- Restart demo without restarting hardware
- Swap between different behaviors easily
- Debug vision separately from robot control
