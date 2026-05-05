#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_PATH="$ROOT_DIR/src/ROBOTIS-OP3-Simulations/op3_webots_ros2/worlds/robotis_op3_extern.wbt"
GC_BIN_DIR="$ROOT_DIR/game_controller-2025.1-x86_64-unknown-linux-gnu"

usage() {
  cat <<'EOF'
Usage:
  ./run_manual_stack.sh <mode>

Modes:
  webots         Run Webots Linux GUI with robotis_op3_extern.wbt
  extern         Run op3_extern_controller
  bridge         Run op3_gc_bridge
  behavior       Run gc_primary_state.launch.py
  all            Run extern + bridge + behavior together (background)
  all_manager    Run extern + op3_manager(sim) + bridge + behavior together
  gamecontroller Run game_controller binary
  check          Quick status checks for key topics
  help           Show this help

Typical startup (in separate terminals):
  ./run_manual_stack.sh webots
  ./run_manual_stack.sh extern
  ./run_manual_stack.sh bridge
  ./run_manual_stack.sh behavior
  ./run_manual_stack.sh all
  ./run_manual_stack.sh all_manager
  ./run_manual_stack.sh gamecontroller
EOF
}

require_file() {
  if [[ ! -f "$1" ]]; then
    echo "ERROR: File not found: $1" >&2
    exit 1
  fi
}

setup_env() {
  export WEBOTS_HOME=/usr/local/webots
  export LD_LIBRARY_PATH="$WEBOTS_HOME/lib/controller:${LD_LIBRARY_PATH:-}"
  set +u
  source /opt/ros/humble/setup.bash
  source "$ROOT_DIR/install/setup.bash"
  set -u
}

mode="${1:-help}"

case "$mode" in
  webots)
    export WEBOTS_HOME=/usr/local/webots
    require_file "$WEBOTS_HOME/webots"
    require_file "$WORLD_PATH"
    exec "$WEBOTS_HOME/webots" "$WORLD_PATH"
    ;;

  extern)
    setup_env
    exec ros2 run op3_webots_ros2 op3_extern_controller
    ;;

  bridge)
    setup_env
    exec ros2 run op3_webots_ros2 op3_gc_bridge
    ;;

  behavior)
    setup_env
    exec ros2 launch gc_behaviour_webots gc_primary_state.launch.py
    ;;

  all)
    setup_env
    mkdir -p "$ROOT_DIR/log"
    ts="$(date +%Y%m%d_%H%M%S)"

    echo "Starting manual ROS stack (extern + bridge + behavior)..."
    ros2 run op3_webots_ros2 op3_extern_controller > "$ROOT_DIR/log/extern_${ts}.log" 2>&1 &
    pid_extern=$!
    ros2 run op3_webots_ros2 op3_gc_bridge > "$ROOT_DIR/log/bridge_${ts}.log" 2>&1 &
    pid_bridge=$!
    ros2 launch gc_behaviour_webots gc_primary_state.launch.py > "$ROOT_DIR/log/behavior_${ts}.log" 2>&1 &
    pid_behavior=$!

    echo "PIDs:"
    echo "  extern   : $pid_extern"
    echo "  bridge   : $pid_bridge"
    echo "  behavior : $pid_behavior"
    echo "Logs:"
    echo "  $ROOT_DIR/log/extern_${ts}.log"
    echo "  $ROOT_DIR/log/bridge_${ts}.log"
    echo "  $ROOT_DIR/log/behavior_${ts}.log"
    echo
    echo "Press Ctrl+C to stop all 3 processes."

    cleanup() {
      echo
      echo "Stopping processes..."
      kill "$pid_behavior" "$pid_bridge" "$pid_extern" 2>/dev/null || true
      wait "$pid_behavior" "$pid_bridge" "$pid_extern" 2>/dev/null || true
      echo "Stopped."
    }
    trap cleanup INT TERM

    wait "$pid_extern" "$pid_bridge" "$pid_behavior"
    ;;

  all_manager)
    setup_env
    mkdir -p "$ROOT_DIR/log"
    ts="$(date +%Y%m%d_%H%M%S)"

    if ! ros2 pkg prefix op3_manager >/dev/null 2>&1; then
      echo "ERROR: ROS package 'op3_manager' not found in current environment." >&2
      echo "Hint: source /opt/ros/humble/setup.bash && source ~/motion_webots/install/setup.bash" >&2
      exit 1
    fi

    echo "Starting manual ROS stack (extern + manager(sim) + bridge + behavior)..."

    ros2 run op3_webots_ros2 op3_extern_controller > "$ROOT_DIR/log/extern_${ts}.log" 2>&1 &
    pid_extern=$!
    sleep 2

    ros2 launch op3_manager op3_simulation.launch.py > "$ROOT_DIR/log/manager_${ts}.log" 2>&1 &
    pid_manager=$!
    sleep 4

    ros2 run op3_webots_ros2 op3_gc_bridge > "$ROOT_DIR/log/bridge_${ts}.log" 2>&1 &
    pid_bridge=$!
    sleep 1

    ros2 launch gc_behaviour_webots gc_primary_state.launch.py > "$ROOT_DIR/log/behavior_${ts}.log" 2>&1 &
    pid_behavior=$!

    echo "PIDs:"
    echo "  extern   : $pid_extern"
    echo "  manager  : $pid_manager"
    echo "  bridge   : $pid_bridge"
    echo "  behavior : $pid_behavior"
    echo "Logs:"
    echo "  $ROOT_DIR/log/extern_${ts}.log"
    echo "  $ROOT_DIR/log/manager_${ts}.log"
    echo "  $ROOT_DIR/log/bridge_${ts}.log"
    echo "  $ROOT_DIR/log/behavior_${ts}.log"
    echo
    echo "Note: Start Webots GUI separately with ./run_manual_stack.sh webots"
    echo "Press Ctrl+C to stop all 4 processes."

    cleanup() {
      echo
      echo "Stopping processes..."
      kill "$pid_behavior" "$pid_bridge" "$pid_manager" "$pid_extern" 2>/dev/null || true
      wait "$pid_behavior" "$pid_bridge" "$pid_manager" "$pid_extern" 2>/dev/null || true
      echo "Stopped."
    }
    trap cleanup INT TERM

    wait "$pid_extern" "$pid_manager" "$pid_bridge" "$pid_behavior"
    ;;

  gamecontroller)
    require_file "$GC_BIN_DIR/game_controller"
    cd "$GC_BIN_DIR"
    exec ./game_controller
    ;;

  check)
    setup_env
    echo "=== /game_controller topics ==="
    ros2 topic list | grep '^/game_controller' || true
    echo
    echo "=== Joint command endpoint ==="
    ros2 topic info -v /robotis_op3/head_pan_position/command || true
    echo
    echo "=== Motion command endpoints ==="
    ros2 topic info -v /robotis/enable_ctrl_module || true
    ros2 topic info -v /robotis/walking/command || true
    ros2 topic info -v /robotis/action/page_num || true
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
