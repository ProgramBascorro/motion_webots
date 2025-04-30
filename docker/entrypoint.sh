#!/bin/bash
set -e

# Activate the virtual environment
source /opt/ros2_venv/bin/activate

# Source the ROS 2 installation setup file
source /opt/ros/jazzy/setup.bash

# Source the workspace setup file
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
  echo "Sourced workspace setup.bash"
else
  echo "Workspace setup.bash not found, only ROS sourced."
fi

# Execute the command passed to the docker run command (or the default CMD)
exec "$@"
