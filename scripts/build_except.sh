#!/bin/bash
cd "$(dirname "$0")/.."

if [ $# -eq 0 ]; then
    echo "Usage: ./build_except.sh <package_name_1> [package_name_2] ..."
    exit 1
fi

echo "============================================"
echo "Building all except selected packages: $@"
echo "============================================"

source /opt/ros/jazzy/setup.bash

colcon build --packages-skip "$@" --symlink-install --event-handlers console_direct+

echo "============================================"
echo "All build except $@ finished."
echo "============================================"