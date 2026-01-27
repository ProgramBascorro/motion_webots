# Fix: Head Stuck Left-Right Oscillation

## Problem Description
The head detects the ball but gets stuck oscillating left-right:
1. **Scanning** → Ball detected → Switch to TRACKING
2. **Head moves toward ball** → Movement too fast/aggressive
3. **Ball leaves camera view** during movement
4. **Ball lost** → Switch back to SCANNING
5. **Repeat** → Stuck in oscillation loop

## Root Cause
**Too aggressive PID gains + no movement limiting = jerky, fast movements that lose the ball**

## Fixes Applied

### 1. Reduced PID Gains (Smoother Tracking)
**File:** `config/head_tracking.yaml`

**Before:**
- `p_gain: 0.65` - Too aggressive, rapid response
- `d_gain: 0.08` - Not enough damping

**After:**
- `p_gain: 0.35` - 46% reduction, smoother response
- `d_gain: 0.10` - 25% increase, better damping

**Effect:** Head moves slower and more smoothly toward ball, less likely to overshoot

### 2. Increased Lost Timeout (Stay Locked Longer)
**File:** `config/head_tracking.yaml`

**Before:**
- `lost_timeout: 2.5` seconds

**After:**
- `lost_timeout: 4.0` seconds (+60%)

**Effect:** If ball briefly disappears during movement, head stays in tracking mode instead of immediately switching back to scanning

### 3. Added Movement Speed Limiting
**File:** `head_tracking_node.py`

**New code:**
```python
# Limit maximum change per iteration to prevent jerky movements
max_delta = 0.05  # Maximum movement per iteration (radians ~2.9°)
pan_cmd = max(-max_delta, min(max_delta, pan_cmd))
tilt_cmd = max(-max_delta, min(max_delta, tilt_cmd))
```

**Effect:**
- Head cannot move more than 0.05 radians (~2.9°) per control cycle
- At 10Hz control rate, maximum speed = 0.5 rad/s (~29°/s)
- Prevents sudden jerky movements that lose the ball

## Expected Behavior Now

### Before Fix (Oscillating):
```
[SCANNING] pan=0.30 → Ball detected at pan=0.50
[TRACKING] pan=0.30 → pan=0.80 (OVERSHOOT!) → Ball lost
[SCANNING] pan=0.80 → Ball detected at pan=0.45
[TRACKING] pan=0.80 → pan=0.20 (OVERSHOOT!) → Ball lost
[SCANNING] pan=0.20 → ... (STUCK IN LOOP)
```

### After Fix (Smooth Tracking):
```
[SCANNING] pan=0.30 → Ball detected at pan=0.50
[TRACKING] pan=0.30 → pan=0.35 → pan=0.40 → pan=0.45 → pan=0.48 (smooth approach)
[TRACKING] Ball centered, tracking stable
[TRACKING] Ball briefly hidden → stays in tracking mode (4s timeout)
[TRACKING] Ball reappears → continues tracking
```

## Summary of Changes

| Parameter | Before | After | Change |
|-----------|--------|-------|--------|
| P gain | 0.65 | 0.35 | -46% (smoother) |
| D gain | 0.08 | 0.10 | +25% (more damping) |
| Lost timeout | 2.5s | 4.0s | +60% (more stable) |
| Max movement | ∞ | 0.05 rad/cycle | NEW (prevents jerks) |
| Max speed | ~unlimited | ~0.5 rad/s | ~29°/s max |

## Testing

After rebuild, test the improvements:

```bash
source install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch op3_ball_localization yolo_scan_only.launch.py
```

**What you should see:**

1. **Smoother transitions**
   - Head moves slowly toward ball
   - No sudden jerky movements

2. **Stable tracking**
   - Once ball is centered, head stays locked on
   - Small smooth adjustments to keep ball centered

3. **No oscillation**
   - Head doesn't swing back and forth
   - Stays in tracking mode even if ball briefly obscured

**Monitor logs:**
```
[SCANNING] pan=0.15, tilt=-0.50, dir=+
[SCANNING] pan=0.30, tilt=-0.50, dir=+
[yolo_to_demo_bridge]: ✓ BALL FOUND: center=(0.45, -0.30), radius=0.06, conf=0.65
[TRACKING] pan=0.35, tilt=-0.48, ball_error=(0.10, -0.02)
[TRACKING] pan=0.40, tilt=-0.46, ball_error=(0.05, -0.01)
[TRACKING] pan=0.43, tilt=-0.45, ball_error=(0.02, 0.00)
[TRACKING] pan=0.44, tilt=-0.45, ball_error=(0.01, 0.00)  ← STABLE
```

## Further Tuning (If Needed)

### If still too fast:
```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py \
  head_tracking_node.p_gain:=0.25 \
  head_tracking_node.max_delta:=0.03
```

### If too slow to react:
```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py \
  head_tracking_node.p_gain:=0.45 \
  head_tracking_node.lost_timeout:=3.0
```

### If oscillating on target:
```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py \
  head_tracking_node.d_gain:=0.15 \
  head_tracking_node.p_gain:=0.30
```

## Build Status
✅ Package rebuilt successfully
✅ Smoother tracking implemented
✅ Movement limiting added
✅ Ready for testing
