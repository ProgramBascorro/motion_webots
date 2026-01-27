# Simple Ball Tracker - Option 3 (Hybrid 2D Approach)

## What This Does

Instead of using 3D PointCloud2, this simplified tracker:

1. **Uses 2D bounding boxes** from `/vision/yolo/detections`
2. **Estimates distance** from bounding box size (bigger box = closer ball)
3. **Uses pixel coordinates** from `/vision/yolo/ball_center` for bearing (left/right angle)
4. **Slower head scanning** to give YOLO more time to detect

## Key Improvements

### Slower Scanning (Better Detection)
- `scan_step: 0.2` (was 0.3) - Smaller steps
- `scan_dwell_sec: 0.8` (was 0.4) - Wait longer at each position
- `lost_timeout: 1.5` (was 0.8) - More tolerant of brief detection gaps

### Distance Estimation Formula
```
distance = (ball_real_diameter * camera_focal_length) / bbox_height_pixels
```

Example:
- Ball diameter: 6.5cm
- Camera focal length: 500px (you need to tune this!)
- Bbox height: 65px
- Distance = (0.065 * 500) / 65 = 0.5m

## How to Run

### 1. Launch YOLO detector
```bash
source install/setup.bash
ros2 launch op3_yolo_vision yolo.launch.py
```

### 2. Launch Simple Ball Tracker
```bash
source install/setup.bash
ros2 launch op3_ball_localization simple_ball_tracker.launch.py
```

## Tuning Distance Estimation

The most important parameter is **`camera_focal_length_px`**. To tune it:

1. Place ball at a known distance (e.g., 0.5m from robot)
2. Run the tracker and check logs for estimated distance
3. If estimated distance is wrong, adjust `camera_focal_length_px`:
   - If estimated distance too large → increase focal_length
   - If estimated distance too small → decrease focal_length

Edit in config: `config/simple_ball_tracker.yaml`

```yaml
# Tune this value!
camera_focal_length_px: 500.0  # Try 400-600 range
```

## Monitoring

```bash
# Check detection rate
ros2 topic hz /vision/yolo/detections

# Check YOLO performance
ros2 topic echo /rosout | grep Performance

# Check tracker logs
ros2 topic echo /rosout | grep "State="
```

## Expected Performance

With optimized YOLO (~18-20 FPS) and slower scanning:
- **Detection should be much more reliable**
- **Head moves slower** so YOLO has time to detect
- **Distance estimation is approximate** but good enough for approach

## Limitations vs 3D Approach

**Pros:**
- ✅ No TF dependencies (faster)
- ✅ No 3D projection math (simpler)
- ✅ Works with current YOLO output

**Cons:**
- ⚠️ Distance estimation is approximate (±10-20% error)
- ⚠️ Assumes ball on ground (no Z coordinate)
- ⚠️ Bearing from pixel position is rough estimate

## If Distance is Inaccurate

You have 3 options:

1. **Tune `camera_focal_length_px`** (easiest, try first!)
2. **Calibrate with test measurements** at multiple distances
3. **Switch back to 3D approach** (Option 1 - more accurate but slower)

## Next Steps

1. Test with real robot
2. Tune `camera_focal_length_px` for your camera
3. Adjust scan speeds if needed
4. Monitor YOLO FPS to ensure it's keeping up

Good luck! 🤖⚽
