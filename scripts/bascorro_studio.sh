#!/usr/bin/env bash

# Allow running via `zsh scripts/bascorro_studio.sh` by re-executing under bash.
# This script is not meant to be sourced.
if [ -n "${BASH_VERSION-}" ]; then
  if [ "${BASH_SOURCE[0]}" != "$0" ]; then
    echo "ERROR: Do not source this script. Run it: scripts/bascorro_studio.sh" >&2
    return 1
  fi
elif [ -n "${ZSH_VERSION-}" ]; then
  case "${ZSH_EVAL_CONTEXT-}" in
    *:file)
      echo "ERROR: Do not source this script. Run it: scripts/bascorro_studio.sh" >&2
      return 1
      ;;
  esac
fi
if [ -z "${BASH_VERSION-}" ]; then
  if command -v bash >/dev/null 2>&1; then
    exec bash "$0" "$@"
  fi
  echo "ERROR: bash is required to run this script." >&2
  exit 1
fi

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DEFAULT="$(cd "$SCRIPT_DIR/.." && pwd)"

WS="${OP3_WS:-$WS_DEFAULT}"
WEB_DIR="${OP3_STUDIO_WEB_DIR:-${OP3_ACTION_WEB_DIR:-$WS/src/bascorro_studio/web}}"
LOG_DIR="${OP3_STUDIO_LOG_DIR:-${OP3_ACTION_WEB_LOG_DIR:-/tmp/bascorro_studio_logs}}"
ASSETS_DIR="${OP3_STUDIO_ASSETS_DIR:-${OP3_ACTION_WEB_ASSETS_DIR:-/tmp/bascorro_studio_assets}}"
ASSETS_PORT="${OP3_STUDIO_ASSETS_PORT:-${OP3_ACTION_WEB_ASSETS_PORT:-8001}}"
WEB_PORT="${OP3_STUDIO_PORT:-${OP3_ACTION_WEB_PORT:-5173}}"
TERMINAL_PORT="${OP3_STUDIO_TERMINAL_PORT:-7681}"
TERMINAL_CREDENTIAL="${OP3_STUDIO_TERMINAL_CREDENTIAL:-}"
ROSBRIDGE_ADDRESS="${OP3_STUDIO_ROSBRIDGE_ADDRESS:-0.0.0.0}"

HOST_IP="${OP3_STUDIO_HOST_IP:-${OP3_ACTION_WEB_HOST_IP:-}}"
if [[ -z "$HOST_IP" ]] && command -v hostname >/dev/null 2>&1; then
  HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
if [[ -z "$HOST_IP" ]]; then
  HOST_IP="localhost"
fi

ROSBRIDGE_URL="${OP3_ROSBRIDGE_URL:-ws://${HOST_IP}:9090}"
ASSETS_URL="${OP3_STUDIO_ASSETS_URL:-${OP3_ACTION_WEB_ASSETS_URL:-http://${HOST_IP}:${ASSETS_PORT}}}"
TERMINAL_URL="${OP3_STUDIO_TERMINAL_URL:-http://${HOST_IP}:${TERMINAL_PORT}}"

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

# Setiap yang di-start_bg adalah `ros2 run` / `ros2 launch`: pembungkus python
# yang node aslinya jadi CUCU. `kill $pid` cuma membunuh pembungkusnya, dan node
# yang tertinggal tetap memegang portnya. Dari situlah gejala "studio kadang
# nyala tapi tombol Walking mati semua": rosbridge sebelumnya masih menempel di
# 9090 atau asset server di 8001, jadi yang baru gagal bind dan diam-diam tidak
# pernah naik. Lihat scripts/lib/proc_tree.sh.
# shellcheck source=lib/proc_tree.sh
source "$SCRIPT_DIR/lib/proc_tree.sh"

cleanup() {
  trap - EXIT INT TERM HUP
  if [[ "${#pids[@]}" -gt 0 ]]; then
    echo "[studio] menghentikan ${#pids[@]} proses latar..."
    proc_kill_tree "${pids[@]}"
  fi

  # pnpm/vite jalan di latar depan, bukan lewat start_bg, jadi tidak ada di
  # $pids -- tanpa sapuan ini port 5173 bisa ikut tertinggal.
  local -a leftovers=() protected=()
  local child
  mapfile -t protected < <(proc_protected_pids)
  while read -r child; do
    if [[ -n "$child" ]] && ! proc_is_protected "$child" "${protected[@]}"; then
      leftovers+=("$child")
    fi
  done < <(proc_descendants "$$")
  if [[ "${#leftovers[@]}" -gt 0 ]]; then
    proc_kill_pids "${leftovers[@]}"
  fi
}
# EXIT saja TIDAK cukup: bash tidak menjalankan trap EXIT kalau ia mati oleh
# sinyal yang tidak di-trap, dan itu justru kasus tersering di sini -- pane tmux
# ditutup, shell-nya dapat SIGHUP, script mati seketika tanpa membereskan apa pun.
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM HUP

if [[ "${OP3_STUDIO_SKIP_BRIDGE:-${OP3_ACTION_WEB_SKIP_BRIDGE:-0}}" != "1" ]]; then
  start_bg "bridge_webots" ros2 run op3_action_editor bridge_webots.py
fi

if [[ "${OP3_STUDIO_SKIP_ROSBRIDGE:-${OP3_ACTION_WEB_SKIP_ROSBRIDGE:-0}}" != "1" ]]; then
  if ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
    if [[ -n "$ROSBRIDGE_ADDRESS" ]]; then
      start_bg "rosbridge" ros2 launch rosbridge_server rosbridge_websocket_launch.xml "address:=$ROSBRIDGE_ADDRESS"
    else
      start_bg "rosbridge" ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    fi
  else
    echo "[studio] rosbridge_server not found; install ros-humble-rosbridge-server"
  fi
fi

start_bg "apply_node" ros2 run bascorro_studio apply_node
start_bg "asset_server" ros2 run bascorro_studio asset_server --port "$ASSETS_PORT" --dir "$ASSETS_DIR"
start_bg "studio_agent" ros2 run bascorro_studio studio_agent
start_bg "terminal_server" ros2 run bascorro_studio terminal_server

cd "$WEB_DIR"
if [[ ! -d node_modules ]]; then
  pnpm install
fi

export VITE_ASSETS_URL="$ASSETS_URL"
export VITE_ROSBRIDGE_URL="$ROSBRIDGE_URL"
# Don't export VITE_TERMINAL_URL - let the web app auto-detect based on hostname
# This allows it to work on both localhost and network IP addresses
# export VITE_TERMINAL_URL="$TERMINAL_URL"

echo "[studio] Vite dev server on port $WEB_PORT"
echo "[studio] Terminal server on port $TERMINAL_PORT"
echo "[studio] ROS bridge URL $ROSBRIDGE_URL"
echo "[studio] ROS bridge bind $ROSBRIDGE_ADDRESS"
echo "[studio] Assets URL $ASSETS_URL"
pnpm dev --host 0.0.0.0 --port "$WEB_PORT"
