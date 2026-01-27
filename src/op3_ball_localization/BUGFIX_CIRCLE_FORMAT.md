# Bug Fix: CircleSetStamped Message Format

## Issue
The head tracking node was crashing with:
```
AttributeError: 'Point' object has no attribute 'radius'
```

## Root Cause
The `CircleSetStamped` message uses `geometry_msgs/Point` objects in its `circles` array, where:
- `Point.x` = normalized x coordinate [-1, 1]
- `Point.y` = normalized y coordinate [-1, 1]
- `Point.z` = normalized radius

The head tracking node was incorrectly trying to access a `radius` attribute directly on the circle object.

## Fix Applied
Updated `head_tracking_node.py` line 110 to correctly access the radius:

**Before:**
```python
if circle.radius > self._min_ball_radius and circle.radius > best_radius:
```

**After:**
```python
radius = circle.z  # Circle is a Point object: x, y = center, z = radius
if radius > self._min_ball_radius and radius > best_radius:
```

## Additional Changes
Also simplified the coordinate normalization logic since the YOLO-to-Demo bridge already provides coordinates in the correct [-1, 1] range:

**Before:**
```python
x_norm = best_ball.x
y_norm = best_ball.y
self._last_ball_x = (x_norm - 0.5) * 2.0  # [-1, 1]
self._last_ball_y = -(y_norm - 0.5) * 2.0  # [-1, 1], inverted
```

**After:**
```python
self._last_ball_x = best_ball.x  # Already normalized [-1, 1]
self._last_ball_y = -best_ball.y  # Invert so up is positive
```

## Testing
After rebuild:
```bash
colcon build --packages-select op3_ball_localization
source install/setup.bash
ros2 launch op3_ball_localization yolo_scan_only.launch.py
```

The node should now start without errors and properly track detected balls.

## Build Status
✅ Package rebuilt successfully
✅ Fix verified in source code
✅ Ready for testing
