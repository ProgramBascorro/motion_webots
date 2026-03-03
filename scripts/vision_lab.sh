#!/usr/bin/env bash

if [ -n "${BASH_VERSION-}" ]; then
  if [ "${BASH_SOURCE[0]}" != "$0" ]; then
    echo "ERROR: Do not source this script. Run it: scripts/vision_lab.sh" >&2
    return 1
  fi
elif [ -n "${ZSH_VERSION-}" ]; then
  case "${ZSH_EVAL_CONTEXT-}" in
    *:file)
      echo "ERROR: Do not source this script. Run it: scripts/vision_lab.sh" >&2
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
VISION_LAB_DIR="${OP3_VISION_LAB_DIR:-$WS/vision_lab}"
SERVER_DIR="${OP3_VISION_LAB_SERVER_DIR:-$VISION_LAB_DIR/apps/server}"
WEB_DIR="${OP3_VISION_LAB_WEB_DIR:-$VISION_LAB_DIR/apps/web}"
VENV_DIR="${OP3_VISION_LAB_VENV_DIR:-$SERVER_DIR/.venv}"
LOG_DIR="${OP3_VISION_LAB_LOG_DIR:-/tmp/vision_lab_logs}"
API_HOST="${OP3_VISION_LAB_API_HOST:-0.0.0.0}"
API_PORT="${OP3_VISION_LAB_API_PORT:-8000}"
WEB_HOST="${OP3_VISION_LAB_WEB_HOST:-0.0.0.0}"
WEB_PORT="${OP3_VISION_LAB_WEB_PORT:-3000}"
API_PUBLIC_URL="${OP3_VISION_LAB_API_URL:-http://localhost:${API_PORT}}"
PYTHON_BIN=""
PIP_BIN=""
USE_DIRECT_PYTHONPATH=0

mkdir -p "$LOG_DIR"

if [[ ! -d "$VISION_LAB_DIR" || ! -d "$SERVER_DIR" || ! -d "$WEB_DIR" ]]; then
  echo "Vision Lab workspace not found under: $VISION_LAB_DIR" >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required to launch Vision Lab." >&2
  exit 1
fi

if ! command -v pnpm >/dev/null 2>&1; then
  echo "pnpm is required to launch Vision Lab." >&2
  exit 1
fi

setup_python_runtime() {
  if [[ -x "$VENV_DIR/bin/python3" && -x "$VENV_DIR/bin/pip" ]]; then
    PYTHON_BIN="$VENV_DIR/bin/python3"
    PIP_BIN="$VENV_DIR/bin/pip"
    return 0
  fi

  if python3 -m venv --help >/dev/null 2>&1; then
    echo "[vision_lab] creating backend virtualenv at $VENV_DIR"
    if python3 -m venv "$VENV_DIR" >/dev/null 2>"$LOG_DIR/venv_create.log"; then
      PYTHON_BIN="$VENV_DIR/bin/python3"
      PIP_BIN="$VENV_DIR/bin/pip"
      return 0
    fi
    echo "[vision_lab] python venv creation unavailable, falling back to system python"
  else
    echo "[vision_lab] python venv module unavailable, falling back to system python"
  fi

  if ! python3 -m pip --version >/dev/null 2>&1; then
    echo "python3 -m pip is required when virtualenv creation is unavailable." >&2
    echo "Install one of: python3-pip or python3.10-venv" >&2
    exit 1
  fi

  PYTHON_BIN="python3"
  PIP_BIN="python3 -m pip"
  USE_DIRECT_PYTHONPATH=1
}

install_backend_deps() {
  if ! python3 - <<'PY' >/dev/null 2>&1
import fastapi
import uvicorn
import numpy
import cv2
import onnxruntime
import multipart
PY
  then
    echo "System python is missing one or more required packages: fastapi uvicorn numpy opencv-python onnxruntime python-multipart" >&2
    echo "Install python3.10-venv for isolated setup, or install the dependencies into your system/user python." >&2
    exit 1
  fi

  if [[ "$USE_DIRECT_PYTHONPATH" -eq 1 ]]; then
    echo "[vision_lab] using system python with PYTHONPATH fallback"
    return 0
  fi

  echo "[vision_lab] ensuring backend dependencies"
  if [[ "$PIP_BIN" == "$VENV_DIR/bin/pip" ]]; then
    if ! "$PIP_BIN" install --no-build-isolation -e "$SERVER_DIR" >"$LOG_DIR/pip_install.log" 2>&1; then
      echo "[vision_lab] backend dependency install failed in virtualenv, falling back to system python" >&2
      sed -n '1,80p' "$LOG_DIR/pip_install.log" >&2 || true
      PYTHON_BIN="python3"
      PIP_BIN="python3 -m pip"
      USE_DIRECT_PYTHONPATH=1
      echo "[vision_lab] using system python with PYTHONPATH fallback"
      return 0
    fi
  else
    if ! python3 -m pip install --user --no-build-isolation -e "$SERVER_DIR" >"$LOG_DIR/pip_install.log" 2>&1; then
      echo "[vision_lab] backend dependency install failed. Log:" >&2
      sed -n '1,200p' "$LOG_DIR/pip_install.log" >&2 || true
      exit 1
    fi
  fi
}

setup_python_runtime
install_backend_deps

if [[ ! -d "$VISION_LAB_DIR/node_modules" ]]; then
  echo "[vision_lab] installing frontend dependencies"
  (cd "$VISION_LAB_DIR" && pnpm install >"$LOG_DIR/pnpm_install.log" 2>&1)
fi

pids=()

cleanup() {
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

echo "[vision_lab] starting FastAPI on ${API_HOST}:${API_PORT}"
(
  cd "$SERVER_DIR"
  if [[ "$USE_DIRECT_PYTHONPATH" -eq 1 ]]; then
    export PYTHONPATH="$SERVER_DIR${PYTHONPATH:+:$PYTHONPATH}"
  fi
  exec $PYTHON_BIN -m uvicorn app.main:app --host "$API_HOST" --port "$API_PORT"
) >"$LOG_DIR/server.log" 2>&1 &
pids+=("$!")

echo "[vision_lab] frontend on http://localhost:${WEB_PORT}"
echo "[vision_lab] backend on ${API_PUBLIC_URL}"
cd "$VISION_LAB_DIR"
export NEXT_PUBLIC_API_BASE_URL="$API_PUBLIC_URL"
pnpm --filter @vision-lab/web dev --hostname "$WEB_HOST" --port "$WEB_PORT"
