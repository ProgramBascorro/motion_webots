# YOLO Vision Simplification - Performance Optimization

## Summary

The `op3_yolo_vision` package has been **drastically simplified** to remove all blocking operations and unnecessary complexity that was causing slow performance on real hardware.

## Key Changes

### Removed Components ❌
1. **TF (Transform) lookups** - Was blocking for 200ms on failures
2. **Camera calibration** - No longer needed for 2D detection
3. **3D ground projection** - Removed ray casting and ground plane projection
4. **PointCloud2 publishing** - Now uses simple 2D BoundingBoxes
5. **Green horizon filtering** - Removed green mask subscription and processing
6. **All TF dependencies** - tf2_ros, tf2_geometry_msgs removed

### What's Left ✅
1. **Pure 2D YOLO detection** - Bounding boxes in pixel coordinates
2. **Ball center tracking** - Normalized (0-1) position in `/vision/yolo/ball_center`
3. **All detections** - Published to `/vision/yolo/detections` (BoundingBoxes msg)
4. **Debug visualization** - Optional debug image with boxes drawn
5. **Performance logging** - Shows FPS and backend in use every 5 seconds

## New Topics

```
/vision/yolo/detections          (soccer_msgs/BoundingBoxes)
/vision/yolo/ball_center         (geometry_msgs/PointStamped)
/vision/yolo/debug               (sensor_msgs/Image) [if publish_debug=true]
```

### BoundingBox Message Format
```
float64 probability      # Detection confidence
int64 xmin, ymin        # Top-left corner (pixels)
int64 xmax, ymax        # Bottom-right corner (pixels)
int64 xbase, ybase      # Base point (center-x, bottom-y)
string class_id         # "ball", "goal post", "robot", etc.
int16 id                # Numeric class ID (0-5)
bool obstacle_detected  # Always false
```

### Ball Center Format
```
Header header
Point point
  float64 x    # Normalized X position (0.0 to 1.0)
  float64 y    # Normalized Y position (0.0 to 1.0)
  float64 z    # Detection confidence score
```

## Performance Improvements

### Why It Was Slow Before:
1. **TF blocking** - 200ms timeout on every frame if TF missing
2. **Single-threaded execution** - All callbacks blocked each other
3. **3D projection calculations** - Heavy math for ground plane projection
4. **Multiple subscriptions** - camera_info, green_mask slowing down
5. **PointCloud2 construction** - Memory intensive

### Why It's Fast Now:
1. **No blocking calls** - Pure computational pipeline
2. **Simplified data flow** - Just: Image → YOLO → BoundingBoxes
3. **Removed subscriptions** - Only subscribes to camera images
4. **Direct 2D output** - Simple message construction
5. **Performance monitoring** - Logs actual FPS achieved

## Expected Performance

- **Webots**: Should still be smooth
- **Real Hardware (CPU)**:
  - 640x640 input: ~5-15 FPS on typical robot CPU
  - 416x416 input: ~15-30 FPS (change `input_size` param)
  - 320x320 input: ~30-50 FPS (lower accuracy)

## Configuration Tips

### For Maximum Speed on Real Hardware:
```yaml
input_size: 320              # Reduce from 640
publish_debug: false         # Disable debug image
use_gpu: false               # Unless you have CUDA
```

### To Enable GPU (if available):
```yaml
use_gpu: true
dnn_backend: cuda
dnn_target: cuda
```

## Build & Test

```bash
cd /home/farhan/Projects/Surgical_lokalisasi_bismillah/motion_webots_farhan_coba
colcon build --packages-select op3_yolo_vision
source install/setup.bash
ros2 launch op3_yolo_vision yolo.launch.py
```

Watch the logs for:
```
[yolo_detector]: Performance: 15.2 ms/frame, 65.8 FPS (backend: opencv/cpu)
```

## Debugging

Check if detections are publishing:
```bash
ros2 topic hz /vision/yolo/detections
ros2 topic echo /vision/yolo/ball_center
```

View debug visualization:
```bash
ros2 run rqt_image_view rqt_image_view /vision/yolo/debug
```

## Code Size Reduction

- **Before**: 850 lines (yolo_detector.cpp)
- **After**: 527 lines
- **Removed**: ~40% of code complexity

Files changed:
- `include/op3_yolo_vision/yolo_detector.hpp` - Simplified class
- `src/yolo_detector.cpp` - Removed TF/3D/horizon code
- `package.xml` - Removed tf2 dependencies
- `CMakeLists.txt` - Removed tf2 dependencies
- `config/yolo.yaml` - Simplified parameters
