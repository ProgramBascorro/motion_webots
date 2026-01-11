#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# Webots env (redundant with Dockerfile ENV but harmless)
export WEBOTS_HOME="${WEBOTS_HOME:-/usr/local/webots}"
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib:$WEBOTS_HOME/lib/controller:${LD_LIBRARY_PATH:-}"
export USER="${USER:-root}"

exec "$@"
