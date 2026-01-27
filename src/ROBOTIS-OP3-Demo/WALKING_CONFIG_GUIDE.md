# OP3 Demo Walking Configuration Guide

## Overview

The `demo.launch.xml` file only configures **head tracking PID gains**:
- `p_gain: 0.45` - Head tracking proportional gain
- `d_gain: 0.045` - Head tracking derivative gain

For **walking parameters**, you need to modify the **C++ source code** and rebuild.

## Walking Parameters Location

### File: `src/ROBOTIS-OP3-Demo/op3_demo/src/soccer/ball_follower.cpp`

Lines 35-50 contain all walking constants:

```cpp
MAX_FB_STEP(40.0 * 0.001),          // Maximum forward/backward step (0.04m = 40mm)
MAX_RL_TURN(15.0 * M_PI / 180),     // Maximum rotation turn (15 degrees)
IN_PLACE_FB_STEP(-3.0 * 0.001),     // In-place step adjustment (-3mm)
MIN_FB_STEP(5.0 * 0.001),           // Minimum forward/backward step (5mm)
MIN_RL_TURN(5.0 * M_PI / 180),      // Minimum rotation (5 degrees)
UNIT_FB_STEP(1.0 * 0.001),          // Step increment unit (1mm)
UNIT_RL_TURN(0.5 * M_PI / 180),     // Turn increment unit (0.5 degrees)
SPOT_FB_OFFSET(0.0 * 0.001),        // Forward/backward offset when standing (0mm)
SPOT_RL_OFFSET(0.0 * 0.001),        // Left/right offset when standing (0mm)
SPOT_ANGLE_OFFSET(0.0),             // Angle offset when standing (0 rad)
hip_pitch_offset_(7.0),             // Hip pitch offset (degrees)
current_x_move_(0.005),             // Initial forward speed (0.005m = 5mm)
curr_period_time_(0.6),             // Walking period time (0.6 seconds)
```

## What Each Parameter Does

### Speed Parameters

| Parameter | Default | Description | To Make Faster | To Make Slower |
|-----------|---------|-------------|----------------|----------------|
| `MAX_FB_STEP` | 40mm | Maximum forward step size | Increase (e.g., 60mm) | Decrease (e.g., 30mm) |
| `current_x_move_` | 5mm | Initial walking speed | Increase (e.g., 10mm) | Decrease (e.g., 3mm) |
| `curr_period_time_` | 0.6s | Step duration | Decrease (faster) | Increase (slower) |

### Turning Parameters

| Parameter | Default | Description | To Turn Faster | To Turn Slower |
|-----------|---------|-------------|----------------|----------------|
| `MAX_RL_TURN` | 15° | Maximum rotation per step | Increase (e.g., 25°) | Decrease (e.g., 10°) |
| `MIN_RL_TURN` | 5° | Minimum rotation | Adjust threshold | Adjust threshold |
| `UNIT_RL_TURN` | 0.5° | Rotation increment | Increase | Decrease |

### Distance/Approach Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MIN_FB_STEP` | 5mm | Minimum step to start walking |
| `UNIT_FB_STEP` | 1mm | Step size increment |
| `IN_PLACE_FB_STEP` | -3mm | Backward adjustment for in-place turns |

### Offset Parameters (Fine-tuning)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPOT_FB_OFFSET` | 0mm | Forward/backward offset when stationary |
| `SPOT_RL_OFFSET` | 0mm | Left/right offset when stationary |
| `SPOT_ANGLE_OFFSET` | 0° | Rotation offset when stationary |
| `hip_pitch_offset_` | 7° | Hip pitch adjustment |

## Common Adjustments

### Make Robot Walk Faster
Edit `ball_follower.cpp` lines 35, 48:
```cpp
MAX_FB_STEP(60.0 * 0.001),    // Was 40.0, now 60mm max step
current_x_move_(0.010),        // Was 0.005, now 10mm initial speed
```

### Make Robot Turn Faster
Edit `ball_follower.cpp` line 36:
```cpp
MAX_RL_TURN(25.0 * M_PI / 180),  // Was 15.0, now 25 degrees
```

### Make Robot More Stable (Slower)
Edit `ball_follower.cpp` lines 35, 50:
```cpp
MAX_FB_STEP(25.0 * 0.001),    // Reduce max step for stability
curr_period_time_(0.8),        // Increase period for slower, stable walking
```

### Adjust Approach Distance (How Close Before Kicking)
The kick distance is calculated dynamically in line 149:
```cpp
double fb_goal = fmin(target_distance * 0.1, MAX_FB_STEP);
```

The robot approaches until ball is close enough. The kick trigger is in `soccer_demo.cpp`.

## Head Tracking Parameters (In Launch File)

These ARE configurable in `demo.launch.xml` (lines 25-26):

```xml
<param name="p_gain" value="0.45" />
<param name="d_gain" value="0.045" />
```

**To adjust head tracking:**
- Increase `p_gain` for more aggressive tracking (e.g., 0.65)
- Increase `d_gain` for better damping (e.g., 0.10)
- Decrease both for smoother, slower tracking

## How to Apply Changes

### Step 1: Edit the Source File
```bash
cd /home/farhan/Projects/Surgical_lokalisasi_bismillah/motion_webots_farhan_coba
nano src/ROBOTIS-OP3-Demo/op3_demo/src/soccer/ball_follower.cpp
```

Edit the constants (lines 35-50).

### Step 2: Rebuild the Package
```bash
colcon build --packages-select op3_demo
source install/setup.bash
```

### Step 3: Restart the Demo
Kill and relaunch:
```bash
# Kill existing demo
pkill -f op_demo_node

# Relaunch
export ROS_DOMAIN_ID=1
ros2 launch op3_demo demo.launch.xml
```

## Alternative: Runtime Walking Parameter Adjustment

You can also adjust walking parameters at runtime using ROS services:

### Get Current Walking Parameters
```bash
export ROS_DOMAIN_ID=1
ros2 service call /robotis/walking/get_params op3_walking_module_msgs/srv/GetWalkingParam "{get_param: true}"
```

### Set Walking Parameters (Example)
```bash
export ROS_DOMAIN_ID=1
ros2 topic pub /robotis/walking/set_params op3_walking_module_msgs/msg/WalkingParam "{
  x_move_amplitude: 0.020,
  y_move_amplitude: 0.000,
  angle_move_amplitude: 0.000,
  period_time: 0.600
}"
```

**Note:** Runtime changes are temporary and reset when the demo restarts.

## Kick Page Numbers

Kick actions are defined in `action_script.yaml`. The default pages:
- Left kick: Page 104
- Right kick: Page 218

These are motion files loaded by the action module.

## Full Demo Configuration Summary

| What to Configure | Where | Type |
|-------------------|-------|------|
| Head tracking PID | `demo.launch.xml` | Launch parameter |
| Walking speed/turning | `ball_follower.cpp` | C++ constant (rebuild required) |
| Kick motions | `action_script.yaml` | Motion file |
| Ball detection | `ball_detector_params.yaml` | Config file |
| Walking period/amplitude | ROS service `/robotis/walking/set_params` | Runtime (temporary) |

## Example: Fast Aggressive Robot

```cpp
// ball_follower.cpp
MAX_FB_STEP(60.0 * 0.001),          // 60mm max step (was 40mm)
MAX_RL_TURN(25.0 * M_PI / 180),     // 25° max turn (was 15°)
current_x_move_(0.012),             // 12mm initial speed (was 5mm)
curr_period_time_(0.5),             // 0.5s period (was 0.6s)
```

```xml
<!-- demo.launch.xml -->
<param name="p_gain" value="0.65" />  <!-- More aggressive tracking -->
<param name="d_gain" value="0.10" />  <!-- Better damping -->
```

## Example: Stable Conservative Robot

```cpp
// ball_follower.cpp
MAX_FB_STEP(25.0 * 0.001),          // 25mm max step (safer)
MAX_RL_TURN(10.0 * M_PI / 180),     // 10° max turn (slower)
current_x_move_(0.003),             // 3mm initial speed (careful)
curr_period_time_(0.8),             // 0.8s period (more stable)
```

```xml
<!-- demo.launch.xml -->
<param name="p_gain" value="0.35" />  <!-- Smoother tracking -->
<param name="d_gain" value="0.08" />  <!-- Less aggressive -->
```

## Troubleshooting

### Robot walks too fast and falls
- Reduce `MAX_FB_STEP` and `current_x_move_`
- Increase `curr_period_time_` for slower steps

### Robot doesn't turn enough
- Increase `MAX_RL_TURN`
- Check ball tracking - if head loses ball, robot won't turn

### Robot oscillates when approaching ball
- Reduce `p_gain` in `demo.launch.xml`
- Increase `d_gain` for better damping
- Reduce `MAX_FB_STEP` for smaller steps near target

### Robot kicks from too far away
- This is controlled in `soccer_demo.cpp` kick trigger logic
- Not easily configurable without code changes

## Summary

**Quick Changes (No Rebuild):**
- Head tracking: Edit `demo.launch.xml` params

**Walking Changes (Requires Rebuild):**
- Speed/turning/approach: Edit `ball_follower.cpp` constants
- Rebuild with `colcon build --packages-select op3_demo`

**Runtime Temporary Changes:**
- Use `/robotis/walking/set_params` service
