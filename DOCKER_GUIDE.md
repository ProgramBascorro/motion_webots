# Docker Guide for OP3 Webots Simulation

This guide provides instructions for running the ROBOTIS OP3 robot simulation using Docker with Webots.

## Prerequisites

### Required Software
- Docker (version 20.10 or higher)
- Docker Compose (version 1.29 or higher)
- X11 server (for GUI display)
  - Linux: Pre-installed
  - macOS: Install [XQuartz](https://www.xquartz.org/)
  - Windows: Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or [Xming](http://www.straightrunning.com/XmingNotes/)

### GPU Support (Optional but Recommended)
For better Webots performance:
- **NVIDIA GPU**: Install [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- **AMD GPU**: Docker should work with mesa drivers already included

## Quick Start

### Method 1: Using Helper Script (Recommended)

The `docker-run.sh` script provides the easiest way to run the simulation:

```bash
# Build the Docker image
./docker-run.sh build

# Run Webots simulation
./docker-run.sh webots

# Or launch an interactive shell
./docker-run.sh shell

# Run OP3 manager in simulation mode
./docker-run.sh manager

# Execute custom ROS2 commands
./docker-run.sh exec ros2 topic list

# Stop all running containers
./docker-run.sh stop
```

### Method 2: Using Docker Compose

#### Production Mode
```bash
# Build the image
docker-compose build

# Run interactive shell
docker-compose run --rm op3_webots

# Run Webots simulation directly
docker-compose up op3_webots_launch

# Run OP3 manager
docker-compose up op3_manager_sim
```

#### Development Mode
For active development with live code editing:

```bash
# Start development container
docker-compose -f docker-compose-dev.yml up -d op3_dev

# Attach to the running container
docker-compose -f docker-compose-dev.yml exec op3_dev bash

# Inside container: build and run
colcon build --symlink-install
source install/setup.bash
ros2 launch op3_webots_ros2 robot_launch.py
```

### Method 3: Direct Docker Commands

```bash
# Build the image
docker build -t op3_webots:humble .

# Run with X11 forwarding
xhost +local:docker
docker run -it --rm \
    --name op3_webots_sim \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume $(pwd)/src:/ws/src:rw \
    --device /dev/dri:/dev/dri \
    --network host \
    --ipc host \
    op3_webots:humble \
    ros2 launch op3_webots_ros2 robot_launch.py
```

## Platform-Specific Setup

### Linux

1. Allow X11 connections:
```bash
xhost +local:docker
```

2. Run the simulation:
```bash
./docker-run.sh webots
```

### macOS

1. Install and start XQuartz:
```bash
brew install --cask xquartz
open -a XQuartz
```

2. In XQuartz preferences, enable "Allow connections from network clients"

3. Allow connections:
```bash
xhost +localhost
```

4. Set DISPLAY environment variable:
```bash
export DISPLAY=host.docker.internal:0
```

5. Run the simulation:
```bash
./docker-run.sh webots
```

### Windows (WSL2)

1. Install VcXsrv or Xming

2. Start X server with:
   - Display number: 0
   - Disable access control: ✓

3. In WSL2 terminal:
```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
./docker-run.sh webots
```

### Windows (Docker Desktop)

1. Install VcXsrv with settings:
   - Multiple windows
   - Display number: 0
   - Start no client
   - Disable access control: ✓

2. PowerShell:
```powershell
set-variable -name DISPLAY -value YOUR_IP:0.0
docker-compose up op3_webots_launch
```

## Available Launch Configurations

### Webots Simulation
Launches Webots with the OP3 robot in a virtual environment:
```bash
ros2 launch op3_webots_ros2 robot_launch.py
```

### OP3 Manager (Simulation Mode)
Runs the robot manager configured for simulation:
```bash
ros2 launch op3_manager op3_simulation.launch.py
```

### Full Demo with GUI
Launches the GUI demo interface:
```bash
ros2 launch op3_gui_demo op3_demo.launch.py
```

## Development Workflow

### Building Code Changes

When using development mode, rebuild your packages:

```bash
# Inside the container
cd /ws
colcon build --packages-select <package_name> --symlink-install
source install/setup.bash
```

### Debugging

```bash
# List running ROS2 nodes
ros2 node list

# Check topics
ros2 topic list
ros2 topic echo /joint_states

# Monitor transforms
ros2 run tf2_tools view_frames

# Check running services
ros2 service list
```

### Accessing Logs

Logs are stored in `/ws/log/` inside the container:

```bash
# View latest logs
ls -lrt /ws/log/latest/

# Tail a specific log
tail -f /ws/log/latest/<package_name>/stdout.log
```

## Troubleshooting

### Issue: "Cannot open display"

**Solution:**
```bash
# On host
xhost +local:docker
echo $DISPLAY  # Should show :0 or :1

# If empty, set it
export DISPLAY=:0
```

### Issue: Webots window is black or not rendering

**Solution:**
1. Check GPU drivers are installed on host
2. Try software rendering:
```bash
# Add to docker run command
--env LIBGL_ALWAYS_SOFTWARE=1
```

### Issue: "Permission denied" for X11 socket

**Solution:**
```bash
# On host
sudo chmod 666 /tmp/.X11-unix/*
xhost +local:docker
```

### Issue: Build fails with rosdep errors

**Solution:**
```bash
# Build with skip rosdep
docker build --build-arg SKIP_ROSDEP=true -t op3_webots:humble .

# Or install dependencies manually inside container
apt-get update
rosdep install --from-paths src --ignore-src -y
```

### Issue: Container runs slow

**Solutions:**
1. Ensure GPU is accessible (check `nvidia-smi` on host)
2. Use `--ipc=host` for better shared memory performance
3. Close unnecessary applications on host
4. Increase Docker resources (CPU/Memory) in Docker Desktop settings

### Issue: Changes to source code not reflected

**Solution:**
When using volume mounts, ensure you:
```bash
# Rebuild after changes
colcon build --packages-select <changed_package> --symlink-install

# Resource the workspace
source install/setup.bash
```

## Performance Optimization

### Using NVIDIA GPU

If you have an NVIDIA GPU:

```bash
# Install nvidia-docker2
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# Run with GPU support (already configured in docker-compose.yml)
docker-compose up op3_webots_launch
```

### Reducing Image Size

```bash
# Multi-stage build to reduce final image size
docker build --target webots -t op3_webots:slim .
```

## Cleaning Up

```bash
# Stop all containers
./docker-run.sh stop

# Or with docker-compose
docker-compose down

# Remove volumes (careful: deletes build artifacts)
docker-compose down -v

# Remove images
docker rmi op3_webots:humble
docker rmi op3_webots:dev

# Clean up Docker system (removes all unused containers, networks, images)
docker system prune -a
```

## Additional Resources

- [Webots Documentation](https://cyberbotics.com/doc/guide/index)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROBOTIS OP3 Manual](http://emanual.robotis.com/docs/en/platform/op3/introduction/)
- [Docker Documentation](https://docs.docker.com/)

## Tips

1. **Persistent volumes**: Build artifacts are stored in named volumes for faster rebuilds
2. **Source mounting**: Development compose file mounts source code for live editing
3. **Network mode host**: Simplifies ROS2 communication between containers
4. **Symlink install**: Use `--symlink-install` for faster Python package development
5. **Parallel builds**: Use `colcon build --parallel-workers 4` to speed up builds
