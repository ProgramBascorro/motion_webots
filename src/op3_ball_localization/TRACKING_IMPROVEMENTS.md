# Head Tracking Improvements

## Issues Fixed
1. **Scan too fast** - Robot head was moving too quickly during scanning
2. **Servo errors** - Scan range was too extreme, causing servo limits errors
3. **Poor ball tracking** - Not tracking ball smoothly enough

## Changes Applied

### 1. Slower Scanning Speed
**File:** `config/head_tracking.yaml`

**Before:**
- `scan_dwell_sec: 0.8` - Too fast, barely pauses at each position

**After:**
- `scan_dwell_sec: 1.5` - Almost 2x slower, gives camera time to detect

### 2. Safer Scan Range (Avoid Servo Errors)
**File:** `config/head_tracking.yaml`

**Pan Range (left-right):**
- Before: `-1.2` to `+1.2` radians (≈ -69° to +69°)
- After: `-0.9` to `+0.9` radians (≈ -52° to +52°)
- **Reduced by 25%** to avoid extreme positions

**Tilt Range (up-down):**
- Before: `-0.6` to `-0.25` radians (≈ -34° to -14°)
- After: `-0.5` to `-0.2` radians (≈ -29° to -11°)
- **Reduced range** for safer operation

**Scan Step:**
- Before: `0.2` radians (≈ 11.5° steps)
- After: `0.15` radians (≈ 8.6° steps)
- **Smaller steps** for smoother scanning

### 3. Better Ball Tracking
**File:** `config/head_tracking.yaml`

**PID Gains (increased for more responsive tracking):**
- `p_gain: 0.45` → `0.65` (+44% stronger response)
- `d_gain: 0.045` → `0.08` (+78% better damping)

**Detection Timeout (stay locked on ball longer):**
- `lost_timeout: 1.5` → `2.5` seconds (+67% longer)
- Ball must be lost for 2.5 seconds before resuming scan

**Ball Detection Threshold:**
- `min_ball_radius: 0.01` → `0.005` (detect smaller/farther balls)

### 4. Code-Level Servo Safety
**File:** `head_tracking_node.py`

**Tracking Mode Limits:**
- Pan: `-1.5` to `+1.5` → `-1.0` to `+1.0`
- Tilt: `-1.0` to `+0.5` → `-0.7` to `+0.3`

**Better Logging:**
- Added `[TRACKING]` and `[SCANNING]` labels
- Increased logging frequency (2.0s → 1.0s throttle)
- Clearer state indication

## Summary of Changes

| Parameter | Before | After | Improvement |
|-----------|--------|-------|-------------|
| Scan dwell time | 0.8s | 1.5s | 87% slower |
| Pan range | ±1.2 rad | ±0.9 rad | 25% safer |
| Tilt range | -0.6 to -0.25 | -0.5 to -0.2 | Safer limits |
| Scan step | 0.2 rad | 0.15 rad | 25% smoother |
| P gain | 0.45 | 0.65 | 44% stronger |
| D gain | 0.045 | 0.08 | 78% better |
| Lost timeout | 1.5s | 2.5s | 67% longer |
| Min ball size | 0.01 | 0.005 | Detects smaller |

## Expected Behavior

### Scanning Mode
- **Speed:** Much slower, pauses 1.5 seconds at each position
- **Range:** Safer, won't hit servo limits
- **Pattern:** Smoother left-right sweeps with up/down alternation
- **Coverage:** Adequate coverage without extreme positions

### Tracking Mode
- **Responsiveness:** More aggressive tracking (higher gains)
- **Stability:** Better damping to prevent oscillation
- **Lock-on:** Stays tracking even if ball briefly hidden (2.5s timeout)
- **Range:** Safe limits prevent servo errors even during tracking

## Testing

After rebuild, test the improvements:

```bash
source install/setup.bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py
```

**What to observe:**

1. **Slower scanning**
   - Head should pause ~1.5 seconds at each position
   - Smoother transitions between positions

2. **No servo errors**
   - No error messages about joint limits
   - Head stays within safe range

3. **Better tracking**
   - When ball detected, head locks on more smoothly
   - Ball stays more centered in camera view
   - Tracking continues even if ball briefly obscured

**Monitor logs:**
```bash
# You should see:
# [SCANNING] pan=0.15, tilt=-0.50, dir=+
# [SCANNING] pan=0.30, tilt=-0.50, dir=+
# ...
# [TRACKING] pan=0.23, tilt=-0.35, ball_error=(0.12, -0.05)
```

## Further Tuning (Optional)

If you need to adjust behavior, you can override parameters at launch:

**Make scan even slower:**
```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py \
  head_tracking_node.scan_dwell_sec:=2.0
```

**Make tracking more aggressive:**
```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py \
  head_tracking_node.p_gain:=0.8 \
  head_tracking_node.d_gain:=0.1
```

**Make tracking less aggressive (smoother but slower):**
```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py \
  head_tracking_node.p_gain:=0.5 \
  head_tracking_node.d_gain:=0.06
```

**Adjust scan range:**
```bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py \
  head_tracking_node.scan_pan_min:=-0.7 \
  head_tracking_node.scan_pan_max:=0.7
```

## Build Status
✅ Package rebuilt successfully
✅ All improvements applied
✅ Ready for testing
