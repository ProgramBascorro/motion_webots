# Quick Reference: YOLO Demo Modes

## Launch Commands

### Scan Only Mode (Safe - No Walking)
```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py
```
**Does:** Head scanning + ball tracking
**Doesn't:** Walk or kick
**Use for:** Safe testing, vision debugging

---

### Full Demo Mode (Complete Behavior)
```bash
ros2 launch op3_ball_localization yolo_full_demo.launch.py
```
**Does:** Head scanning + ball tracking + walking + kicking
**Doesn't:** Nothing - full behavior
**Use for:** Complete soccer demonstration

---

### Original Mode (Backward Compatibility)
```bash
ros2 launch op3_ball_localization yolo_demo.launch.py
```
**Same as:** Full Demo Mode
**Use for:** Legacy scripts/documentation

---

## Prerequisites

**Always launch manager + camera first:**
```bash
./script.sh
# Select option: "op3_manager + usb_cam"
```

**Then in a new terminal:**
```bash
source install/setup.bash
ros2 launch op3_ball_localization <your_choice>.launch.py
```

---

## Quick Checks

### Is ball being detected?
```bash
ros2 topic echo /ball_detector_node/circle_set
```

### Is head moving?
```bash
ros2 topic echo /robotis/head_control/set_joint_states
```

### Is robot trying to walk?
```bash
ros2 topic hz /robotis/walking/command
# Scan only: 0 Hz (no walking)
# Full demo: >0 Hz when ball detected
```

### What nodes are running?
```bash
ros2 node list | grep -E "(yolo|bridge|demo|head)"
```

---

## Configuration Override

### Change PID gains for scan only:
```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py \
  head_tracking_node.p_gain:=0.5 \
  head_tracking_node.d_gain:=0.05
```

### Change scan pattern:
```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py \
  head_tracking_node.scan_step:=0.3 \
  head_tracking_node.scan_dwell_sec:=0.5
```

---

## Troubleshooting

| Problem | Check | Solution |
|---------|-------|----------|
| Head doesn't move | Manager running? | Launch via script.sh first |
| Ball not detected | YOLO topic? | `ros2 topic hz /vision/yolo/ball_center` |
| Robot walks in scan mode | Wrong launch file? | Use `yolo_scan_only.launch.py` |
| Robot doesn't walk in full mode | Ball detected? | Check `/ball_detector_node/circle_set` |

---

## File Locations

**Launch files:**
- `/launch/yolo_scan_only.launch.py`
- `/launch/yolo_full_demo.launch.py`
- `/launch/yolo_demo.launch.py` (original)

**Config files:**
- `/config/head_tracking.yaml`

**Documentation:**
- `YOLO_LAUNCH_MODES.md` (detailed guide)
- `IMPLEMENTATION_SUMMARY.md` (technical details)
- `QUICK_REFERENCE.md` (this file)

---

## Component Breakdown

### Scan Only Mode
```
Camera → YOLO → Bridge → Head Tracking → Head Control
                                            (no walking)
```

### Full Demo Mode
```
Camera → YOLO → Bridge → Demo Node → Head + Walking + Kick
```

---

## When to Use Each Mode

| Mode | Use Case |
|------|----------|
| **Scan Only** | Testing on a stand/table, debugging vision, safe demos |
| **Full Demo** | Complete robot demo, soccer matches, full behavior |
| **Original** | Existing scripts need it, backward compatibility |

---

## Build & Install

After any changes:
```bash
colcon build --packages-select op3_ball_localization
source install/setup.bash
```

---

## For More Information

- **User Guide:** See `YOLO_LAUNCH_MODES.md`
- **Technical Details:** See `IMPLEMENTATION_SUMMARY.md`
- **Source Code:** See `/op3_ball_localization/head_tracking_node.py`
