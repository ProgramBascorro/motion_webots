#!/usr/bin/env bash
set -e

# A sourced script inherits the caller's positional parameters, and OpenVINO's
# setupvars.sh parses + shifts them away. That leaves "$@" empty here, so the
# final `exec "$@"` runs nothing and the container exits 0 the instant it starts
# (verified on CHRONUS: `docker run --rm op3-webots-ros2:humble echo HELLO`
# prints "[docker-entrypoint] launching:" with nothing after it). Only the amd64
# images actually ship setupvars.sh -- this robot is arm64 and uses the pip
# openvino wheel, so it does not have the file today -- but the entrypoint must
# not depend on that. Stash the command Docker handed us and restore it right
# before exec.
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
