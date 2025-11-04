# Build Notes

## Known Issues and Solutions

### ROS1 (Catkin) Packages

The following packages are ROS1 packages and are automatically ignored during build:
- `ros_madplay_player` - Audio player using madplay (ROS1)
- `ros_mpg321_player` - Audio player using mpg321 (ROS1)

These packages have `COLCON_IGNORE` files created automatically during Docker build.

### Missing Dependencies

Some packages reference dependencies that have different names in ROS2 or need manual installation:

| Package | Missing Dep | Solution |
|---------|-------------|----------|
| `robotis_math` | `eigen3` | Installed as `libeigen3-dev` |
| `humanoid_localization` | `Eigen3` | Installed as `libeigen3-dev` |
| `footstep_planner` | `opencv` | Installed as `libopencv-dev` |
| `humanoid_planner_2d` | `opencv4` | Installed as `libopencv-dev` |
| `op3_online_walking_module` | `orocos_kdl` | Built-in with ROS2 |
| `op3_ball_detector` | `uvc_camera` | Use `usb_cam` instead (ROS2) |
| `op3_navigation` | `map_server` | Use `nav2_map_server` (Navigation2) |
| `op3_kinematics_dynamics` | `cmake_modules` | Not needed in ROS2 |

## Docker Build Process

The Dockerfile handles these issues automatically:

1. **Installs missing system dependencies**:
   ```bash
   libeigen3-dev
   libopencv-dev
   ros-humble-cv-bridge
   ros-humble-image-transport
   ros-humble-navigation2
   ros-humble-nav2-bringup
   ```

2. **Ignores ROS1 packages**:
   - Creates `COLCON_IGNORE` files for catkin-based packages

3. **Skips problematic rosdep keys**:
   - Uses `--skip-keys` to ignore unavailable dependencies

4. **Builds with fallback**:
   - First tries normal build
   - Falls back to `--continue-on-error` if needed

## Manual Build (Native/Inside Container)

If building manually:

```bash
# 1. Source ROS2
source /opt/ros/humble/setup.bash

# 2. Install system dependencies
sudo apt-get update
sudo apt-get install -y libeigen3-dev libopencv-dev \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-navigation2 ros-humble-nav2-bringup

# 3. Ignore ROS1 packages
touch src/ROBOTIS-Utility/ros_madplay_player/COLCON_IGNORE
touch src/ROBOTIS-Utility/ros_mpg321_player/COLCON_IGNORE

# 4. Install ROS dependencies (skipping problematic ones)
rosdep install --from-paths src --ignore-src -y \
    --skip-keys="catkin roscpp opencv opencv4 Eigen3 eigen3 cmake_modules orocos_kdl uvc_camera map_server"

# 5. Build
colcon build --symlink-install \
    --packages-skip ros_madplay_player ros_mpg321_player

# Or use the helper script
./scripts/build-workspace.sh
```

## Build Script Usage

The `scripts/build-workspace.sh` script handles all the above automatically:

```bash
# Build all packages
./scripts/build-workspace.sh

# Clean and build
./scripts/build-workspace.sh --clean

# Build specific packages
./scripts/build-workspace.sh --packages "op3_manager op3_walking_module"

# Use more parallel jobs
./scripts/build-workspace.sh --parallel 8
```

## Packages Successfully Built

After fixing the above issues, these core packages build successfully:

**Motion Control:**
- ✅ `op3_manager`
- ✅ `op3_walking_module`
- ✅ `op3_online_walking_module`
- ✅ `op3_action_module`
- ✅ `op3_balance_control`
- ✅ `op3_kinematics_dynamics`
- ✅ `op3_base_module`
- ✅ `op3_head_control_module`
- ✅ `op3_direct_control_module`
- ✅ `op3_tuning_module`

**Simulation:**
- ✅ `op3_webots_ros2`
- ✅ `op3_gazebo_ros2`

**Framework:**
- ✅ `robotis_controller`
- ✅ `robotis_device`
- ✅ `robotis_framework_common`
- ✅ `robotis_math`
- ✅ `open_cr_module`

**Messages:**
- ✅ All message packages (`*_msgs`)

**Vision:**
- ✅ `op3_ball_detector` (with usb_cam instead of uvc_camera)
- ✅ `face_detection`

**Tools:**
- ✅ `op3_action_editor`
- ✅ `op3_gui_demo`
- ✅ `op3_camera_setting_tool`
- ✅ `op3_offset_tuner_server`
- ✅ `op3_offset_tuner_client`

**Not Built (ROS1 only):**
- ❌ `ros_madplay_player` - Audio player, not essential
- ❌ `ros_mpg321_player` - Audio player, not essential

## Testing the Build

After building, test with:

```bash
# Source workspace
source install/setup.bash

# Test basic ROS2 functionality
ros2 pkg list | grep op3

# Test Webots simulation launch
ros2 launch op3_webots_ros2 robot_launch.py

# Test OP3 manager
ros2 launch op3_manager op3_simulation.launch.py
```

## Troubleshooting Build Errors

### Error: "Package 'X' not found"
**Solution**: Check if package has COLCON_IGNORE or was skipped. Use `--packages-select X` to build specifically.

### Error: CMake can't find package
**Solution**: Install missing system dependency or ROS2 package with `apt-get install`.

### Error: Multiple packages provide 'X'
**Solution**: This is usually harmless, the build will continue.

### Build is slow
**Solution**: Use `--parallel-workers N` where N is number of CPU cores (e.g., 8).

### Out of memory during build
**Solution**: Reduce parallel workers: `colcon build --parallel-workers 2`
