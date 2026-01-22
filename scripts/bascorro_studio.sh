#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DEFAULT="$(cd "$SCRIPT_DIR/.." && pwd)"

WS="${OP3_WS:-$WS_DEFAULT}"
WEB_DIR="${OP3_STUDIO_WEB_DIR:-${OP3_ACTION_WEB_DIR:-$WS/src/bascorro_studio/web}}"
LOG_DIR="${OP3_STUDIO_LOG_DIR:-${OP3_ACTION_WEB_LOG_DIR:-/tmp/bascorro_studio_logs}}"
ASSETS_DIR="${OP3_STUDIO_ASSETS_DIR:-${OP3_ACTION_WEB_ASSETS_DIR:-/tmp/bascorro_studio_assets}}"
ASSETS_PORT="${OP3_STUDIO_ASSETS_PORT:-${OP3_ACTION_WEB_ASSETS_PORT:-8001}}"
WEB_PORT="${OP3_STUDIO_PORT:-${OP3_ACTION_WEB_PORT:-5173}}"
ROSBRIDGE_URL="${OP3_ROSBRIDGE_URL:-ws://localhost:9090}"
ASSETS_URL="${OP3_STUDIO_ASSETS_URL:-${OP3_ACTION_WEB_ASSETS_URL:-http://localhost:${ASSETS_PORT}}}"

mkdir -p "$LOG_DIR"

if [[ ! -d "$WEB_DIR" ]]; then
  echo "Web directory not found: $WEB_DIR" >&2
  exit 1
fi

if ! command -v pnpm >/dev/null 2>&1; then
  echo "pnpm is required. Install it first: npm install -g pnpm" >&2
  exit 1
fi

pids=()

start_bg() {
  local name="$1"
  shift
  local logfile="$LOG_DIR/${name}.log"
  "$@" >"$logfile" 2>&1 &
  pids+=("$!")
  echo "[studio] started $name (log: $logfile)"
}

cleanup() {
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

if [[ "${OP3_STUDIO_SKIP_BRIDGE:-${OP3_ACTION_WEB_SKIP_BRIDGE:-0}}" != "1" ]]; then
  start_bg "bridge_webots" ros2 run op3_action_editor bridge_webots.py
fi

if [[ "${OP3_STUDIO_SKIP_ROSBRIDGE:-${OP3_ACTION_WEB_SKIP_ROSBRIDGE:-0}}" != "1" ]]; then
  if ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
    start_bg "rosbridge" ros2 launch rosbridge_server rosbridge_websocket_launch.xml
  else
    echo "[studio] rosbridge_server not found; install ros-humble-rosbridge-server"
  fi
fi

start_bg "apply_node" ros2 run bascorro_studio apply_node
start_bg "asset_server" ros2 run bascorro_studio asset_server --port "$ASSETS_PORT" --dir "$ASSETS_DIR"
start_bg "studio_agent" ros2 run bascorro_studio studio_agent

cd "$WEB_DIR"
if [[ ! -d node_modules ]]; then
  pnpm install
fi

export VITE_ASSETS_URL="$ASSETS_URL"
export VITE_ROSBRIDGE_URL="$ROSBRIDGE_URL"

echo "[studio] Vite dev server on port $WEB_PORT"
exec pnpm dev --host 0.0.0.0 --port "$WEB_PORT"
