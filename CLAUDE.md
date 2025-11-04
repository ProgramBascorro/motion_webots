# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS 2 Humble workspace for the ROBOTIS OP3 humanoid robot with Webots simulation integration. The project includes motion control (walking, balance, actions), vision systems (ball detection), hardware interfaces (OpenCR), and simulation tools for RoboCup soccer development.

## Build System

This is a standard ROS 2 workspace using `colcon` build system with `ament_cmake` packages.

### Build Commands

```bash
# From workspace root (/home/myudak/project/motion_webots)
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Build specific packages
colcon build --packages-select op3_manager op3_walking_module

# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Sourcing the Workspace

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Key Launch Files

### Simulation

```bash
# Launch OP3 manager in simulation mode
ros2 launch op3_manager op3_simulation.launch.py

# Launch Webots simulation (if webots integration is configured)
# Check src/ROBOTIS-OP3-Simulations/op3_webots_ros2/launch/ for available launch files

# Launch Gazebo simulation
ros2 launch op3_gazebo_ros2 robot_sim.launch.py
```

### Hardware (Physical Robot)

```bash
# Launch OP3 manager for hardware
ros2 launch op3_manager op3_manager.launch.py

# Launch full bringup with camera
ros2 launch op3_bringup op3_bringup.launch.py

# Launch OpenCR hardware interface
ros2 launch open_cr_module open_cr.launch.py
```

### Tools and Tuning

```bash
# Action editor for creating/editing motion sequences
ros2 launch op3_action_editor [launch_file]

# GUI demo interface
ros2 launch op3_gui_demo op3_demo.launch.py

# Camera settings tool
ros2 launch op3_camera_setting_tool [launch_file]
```

## Architecture

### Core Control System

- **op3_manager**: Central robot manager that loads and coordinates all control modules. Manages the real-time control loop and handles module switching. This is the main entry point for both simulation and hardware.

- **robotis_controller**: Framework-level controller that manages the control pipeline, sensor data processing, and actuator commands.

- **robotis_device**: Hardware abstraction layer for sensors and actuators.

- **open_cr_module**: Hardware interface to OpenCR board (IMU, buttons, LED). In simulation this may be mocked/emulated.

### Motion Modules

The OP3 uses a modular motion control system where different "modules" can control different joints:

- **op3_base_module**: Basic pose control and initialization.

- **op3_walking_module**: Static/scripted walking patterns.

- **op3_online_walking_module**: Real-time adaptive walking with online trajectory generation. More advanced than walking_module.

- **op3_action_module**: Executes pre-recorded motion sequences (kick, get-up, etc.) from YAML action scripts (`src/ROBOTIS-OP3-Tools/op3_action_editor/config/editor_script.yaml`).

- **op3_head_control_module**: Independent head control for tracking.

- **op3_direct_control_module**: Direct joint position control.

- **op3_tuning_module**: Joint offset and pose tuning.

### Motion Support Libraries

- **op3_kinematics_dynamics**: Forward/inverse kinematics and dynamics calculations.

- **op3_balance_control**: Balance control algorithms for stable standing and dynamic motion.

- **robotis_math**: Mathematical utilities (transformations, filters, trajectory generation).

### Vision System

- **op3_ball_detector**: Ball detection node (interfaces defined in `op3_ball_detector_msgs`).

- **op3_camera_setting_tool**: Camera parameter configuration.

- **face_detection**: Face tracking capabilities.

Note: Vision processing implementation may be external (OpenCV, YOLO, etc.).

### Localization and Navigation

- **op3_localization**: Robot position tracking on field.

- **humanoid_navigation**: Footstep planning and navigation for humanoid robots (includes footstep_planner, humanoid_localization, gridmap_2d).

### Configuration Files

- Robot hardware config: `src/ROBOTIS-OP3/op3_manager/config/dxl_init_OP3.yaml`
- Joint offsets: `src/ROBOTIS-OP3/op3_manager/config/offset.yaml`
- Walking parameters: `src/ROBOTIS-OP3/op3_online_walking_module/config/walking_parm.yaml`
- Action sequences: `src/ROBOTIS-OP3-Tools/op3_action_editor/config/editor_script.yaml`
- Balance gains: `src/ROBOTIS-OP3/op3_online_walking_module/config/balance_gain.yaml`

## Docker Support

Docker setup is available for easier development and deployment. See `DOCKER_GUIDE.md` for comprehensive instructions.

### Quick Docker Commands

```bash
# Using helper script (recommended)
./docker-run.sh build          # Build the image
./docker-run.sh webots         # Run Webots simulation
./docker-run.sh shell          # Interactive shell
./docker-run.sh manager        # Run OP3 manager

# Using docker-compose
docker-compose build           # Build production image
docker-compose up op3_webots_launch  # Run simulation

# Development mode (with live code editing)
docker-compose -f docker-compose-dev.yml up -d op3_dev
docker-compose -f docker-compose-dev.yml exec op3_dev bash
```

### Build Arguments

```bash
# Build with custom arguments
docker build --build-arg SKIP_APT=true -t op3_webots .
docker build --build-arg SKIP_ROSDEP=true -t op3_webots .
docker build --build-arg SKIP_COLCON=true -t op3_webots .
```

The Dockerfile uses ROS 2 Humble and integrates Webots R2023b. It includes X11 forwarding for GUI support and GPU acceleration capabilities.

## Dependencies

- ROS 2 Humble
- Webots R2023b (for simulation)
- DynamixelSDK (motor communication)
- Standard ROS 2 packages: geometry_msgs, sensor_msgs, std_msgs
- usb_cam (for hardware camera interface)

Dependencies are managed via rosdep and defined in package.xml files.

## Module Communication

Modules communicate with op3_manager via:
- `robotis_controller_msgs`: Enable/disable modules, set joint control modes
- `op3_walking_module_msgs`, `op3_online_walking_module_msgs`: Walking commands
- `op3_action_module_msgs`: Action playback commands

The manager ensures only compatible modules control joints simultaneously.

## Repository Structure

```
src/
├── ROBOTIS-OP3/                 # Core OP3 packages (manager, modules)
├── ROBOTIS-OP3-Common/          # URDF, description files
├── ROBOTIS-OP3-Demo/            # Demo applications, bringup
├── ROBOTIS-OP3-msgs/            # Message definitions
├── ROBOTIS-OP3-Simulations/     # Gazebo and Webots integration
├── ROBOTIS-OP3-Tools/           # Tuning and configuration tools
├── ROBOTIS-Framework/           # Controller framework (device, controller)
├── ROBOTIS-Framework-msgs/      # Framework messages
├── ROBOTIS-Math/                # Math utilities
├── ROBOTIS-Utility/             # Audio/utility packages
├── DynamixelSDK/                # Dynamixel motor SDK
├── humanoid_navigation/         # Footstep planning, localization
├── humanoid_msgs/               # Navigation messages
└── face_detection/              # Face detection capabilities
```
