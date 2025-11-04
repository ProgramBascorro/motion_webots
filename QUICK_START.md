# Quick Start Guide

## Choose Your Build Method

### Method 1: Development Build (Recommended - Faster)

This method builds the Docker image without building the workspace, allowing you to build manually inside the container. This is **much faster** and gives you more control.

```bash
# 1. Build the development image (faster - no workspace build)
./docker-run.sh build-dev

# 2. Start an interactive shell
./docker-run.sh shell-dev

# 3. Inside the container, build the workspace
./scripts/build-workspace.sh

# 4. Run the simulation
ros2 launch op3_webots_ros2 robot_launch.py
```

**Advantages:**
- ✅ Much faster Docker build (no waiting for colcon build)
- ✅ See build errors clearly
- ✅ Rebuild only changed packages
- ✅ Better for development iteration

### Method 2: Full Build (All-in-One)

This method builds everything during Docker image creation. Takes longer but workspace is ready to use.

```bash
# 1. Build the full image (slower - includes workspace build)
./docker-run.sh build

# 2. Run simulation directly
./docker-run.sh webots
```

**Advantages:**
- ✅ One-step process after image is built
- ✅ Workspace pre-built and ready
- ✅ Good for deployment

## Troubleshooting Build Issues

If you encounter build errors with either method:

### Inside Container Build Errors

```bash
# Enter container
./docker-run.sh shell-dev

# Try clean build
./scripts/build-workspace.sh --clean

# Build specific packages only
./scripts/build-workspace.sh --packages "op3_manager op3_walking_module"

# Build with more verbose output
colcon build --symlink-install \
    --packages-skip ros_madplay_player ros_mpg321_player \
    --event-handlers console_direct+
```

### Check What's Available

```bash
# List packages in workspace
colcon list

# Check for packages with COLCON_IGNORE
find src -name COLCON_IGNORE

# Check installed ROS2 packages
apt list --installed | grep ros-humble
```

## Common Issues

### "Some packages failed to build"

This is usually okay! The essential packages for simulation typically build successfully. Check which packages failed:

```bash
# Inside container after build
cat log/latest_build/events.log | grep -A 5 "Failed"
```

**Non-essential packages that may fail:**
- `ros_madplay_player` - Audio (ROS1 only)
- `ros_mpg321_player` - Audio (ROS1 only)
- Some navigation packages - Not needed for basic simulation

**Essential packages that should build:**
- `op3_manager` ✅
- `op3_webots_ros2` ✅
- `op3_walking_module` ✅
- `op3_online_walking_module` ✅
- `robotis_controller` ✅

### Build is Very Slow

```bash
# Use more parallel jobs (default is 4)
./scripts/build-workspace.sh --parallel 8

# Or directly
colcon build --symlink-install --parallel-workers 8
```

### Out of Memory During Build

```bash
# Reduce parallel workers
./scripts/build-workspace.sh --parallel 2
```

## Next Steps After Successful Build

### Test the Simulation

```bash
# Inside container with built workspace
source install/setup.bash
ros2 launch op3_webots_ros2 robot_launch.py
```

### Check ROS2 Communication

```bash
# In another terminal/container
ros2 topic list
ros2 topic echo /joint_states
ros2 node list
```

### Run Different Modules

```bash
# OP3 Manager
ros2 launch op3_manager op3_simulation.launch.py

# GUI Demo
ros2 launch op3_gui_demo op3_demo.launch.py

# Ball Detector
ros2 launch op3_ball_detector ball_detector_from_usb_cam.launch.py
```

## File Structure

```
motion_webots/
├── Dockerfile              # Full build (workspace included)
├── Dockerfile.dev          # Dev build (no workspace build)
├── docker-compose.yml      # Production setup
├── docker-compose-dev.yml  # Development setup
├── docker-run.sh           # Helper script (recommended)
├── Makefile               # Make shortcuts
├── scripts/
│   └── build-workspace.sh # Build helper
├── BUILD_NOTES.md         # Detailed build information
├── DOCKER_GUIDE.md        # Comprehensive Docker guide
└── QUICK_START.md         # This file
```

## Makefile Shortcuts

If you prefer using make:

```bash
make dev-build    # Build dev image
make dev-up       # Start dev container
make dev-shell    # Attach to container
make dev-down     # Stop container

make build        # Build full image
make webots       # Run simulation
make stop         # Stop all
```

## Need Help?

1. Check `BUILD_NOTES.md` for detailed build information
2. Check `DOCKER_GUIDE.md` for comprehensive Docker instructions
3. Check `CLAUDE.md` for architecture and system overview

## Recommended Workflow

**For Development:**
```bash
./docker-run.sh build-dev              # Once
./docker-run.sh shell-dev              # Start container
# Inside: ./scripts/build-workspace.sh  # Build/rebuild as needed
```

**For Quick Testing:**
```bash
./docker-run.sh build                  # Once (takes longer)
./docker-run.sh webots                 # Run anytime
```

**For Continuous Development:**
```bash
docker-compose -f docker-compose-dev.yml up -d
docker-compose -f docker-compose-dev.yml exec op3_dev bash
# Edit code on host, rebuild inside container
```
