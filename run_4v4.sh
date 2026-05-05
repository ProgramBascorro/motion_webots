#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GC_BIN="$ROOT_DIR/game_controller-2025.1-x86_64-unknown-linux-gnu/game_controller"

# Reduce FastDDS shared-memory lock errors on heavy multi-node launches.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

usage() {
  cat <<'EOF'
Usage:
  ./run_4v4.sh <mode>

Modes:
  sim            Run 4v4 strategy stack + Webots GUI (YOLO off)
  sim-yolo       Run 4v4 strategy stack + Webots GUI (YOLO basic perception on)
  gc             Run GameController UI
  clean-view     Remove Webots .wbproj files (fix blank/empty viewport issue)
  help           Show this help

Examples:
  ./run_4v4.sh sim
  ./run_4v4.sh sim-yolo
  ./run_4v4.sh gc
EOF
}

setup_env() {
  # ROS setup scripts may reference optional vars that are unset.
  # Temporarily disable nounset to avoid "unbound variable" failures.
  set +u
  source /opt/ros/humble/setup.bash
  source "$ROOT_DIR/install/setup.bash"
  set -u
}

mode="${1:-help}"

case "$mode" in
  sim)
    setup_env
    exec ros2 launch op3_soccer_core soccer_4v4_gui_full_yolo.launch.py \
      use_yolo:=false
    ;;

  sim-yolo)
    setup_env
    exec ros2 launch op3_soccer_core soccer_4v4_gui_full_yolo.launch.py \
      use_yolo:=true \
      yolo_input_size:=416 \
      yolo_frame_skip:=2 \
      yolo_publish_debug:=false
    ;;

  gc)
    if [[ ! -x "$GC_BIN" ]]; then
      echo "ERROR: GameController binary not found: $GC_BIN" >&2
      exit 1
    fi
    exec "$GC_BIN"
    ;;

  clean-view)
    WORLD_DIR="$ROOT_DIR/src/ROBOTIS-OP3-Simulations/op3_webots_ros2/worlds"
    ts="$(date +%Y%m%d_%H%M%S)"
    for f in "$WORLD_DIR"/.robotis_op3_extern*.wbproj; do
      [[ -e "$f" ]] || continue
      mv "$f" "${f}.bak_${ts}"
      echo "Moved: $f -> ${f}.bak_${ts}"
    done
    ;;

  help|-h|--help)
    usage
    ;;

  *)
    echo "ERROR: Unknown mode: $mode" >&2
    usage
    exit 1
    ;;
esac
