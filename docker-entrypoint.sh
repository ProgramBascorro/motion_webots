#!/usr/bin/env bash
set -e

# A sourced script inherits the caller's positional parameters, and OpenVINO's
# setupvars.sh parses + shifts them away. That left "$@" empty here, so the
# final `exec "$@"` ran nothing and the container exited 0 the instant it
# started. Verified on this robot's image: `docker run --rm
# op3-webots-ros2:humble echo HELLO` prints "[docker-entrypoint] launching:"
# with nothing after it, and HELLO never appears. Stash the command Docker
# handed us and restore it right before exec.
entrypoint_cmd=("$@")

if [ -n "${OPENVINO_ROOT:-}" ] && [ -f "${OPENVINO_ROOT}/setupvars.sh" ]; then
  # setupvars.sh exports the runtime library paths and CMake hints expected by OpenVINO.
  # shellcheck disable=SC1090
  if ! source "${OPENVINO_ROOT}/setupvars.sh"; then
    echo "[docker-entrypoint] Warning: OpenVINO setupvars.sh returned non-zero, continuing anyway" >&2
  fi
fi

if ! source /opt/ros/humble/setup.bash; then
  echo "[docker-entrypoint] Error: failed to source /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [ -f /ros2_ws/install/setup.bash ]; then
  if ! source /ros2_ws/install/setup.bash; then
    echo "[docker-entrypoint] Warning: /ros2_ws/install/setup.bash returned non-zero, continuing anyway" >&2
  fi
fi

# Webots env (redundant with Dockerfile ENV but harmless)
export WEBOTS_HOME="${WEBOTS_HOME:-/usr/local/webots}"
export LD_LIBRARY_PATH="$WEBOTS_HOME/lib:$WEBOTS_HOME/lib/controller:${LD_LIBRARY_PATH:-}"
export USER="${USER:-root}"

set -- "${entrypoint_cmd[@]}"

echo "[docker-entrypoint] launching: $*" >&2
exec "$@"
