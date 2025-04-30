#!/bin/bash

# Navigate to the workspace root (adjust path if script is run from elsewhere)
cd "$(dirname "$0")/.." # Go up one level from scripts/ to ros2_ws/

echo "============================================"
echo "Building ALL packages in workspace..."
echo "============================================"

# Source ROS environment (important!)
source /opt/ros/jazzy/setup.bash

# Run colcon build (add your preferred options)
colcon build --symlink-install

echo "============================================"
echo "Build finished."
echo "============================================"