#!/usr/bin/env bash

# Allow running via `zsh scripts/op3_action_web.sh` by re-executing under bash.
# This script is not meant to be sourced.
if [ -n "${BASH_VERSION-}" ]; then
  if [ "${BASH_SOURCE[0]}" != "$0" ]; then
    echo "ERROR: Do not source this script. Run it: scripts/op3_action_web.sh" >&2
    return 1
  fi
elif [ -n "${ZSH_VERSION-}" ]; then
  case "${ZSH_EVAL_CONTEXT-}" in
    *:file)
      echo "ERROR: Do not source this script. Run it: scripts/op3_action_web.sh" >&2
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
echo "[action_web] Deprecated: use bascorro_studio.sh instead."
exec "$SCRIPT_DIR/bascorro_studio.sh" "$@"
