#!/usr/bin/env bash

# Allow running via `zsh script.sh` (or any non-bash shell) by re-executing under bash.
# This script is not meant to be sourced.
if [ -n "${BASH_VERSION-}" ]; then
  if [ "${BASH_SOURCE[0]}" != "$0" ]; then
    echo "ERROR: Do not source this script. Run it: ./script.sh [args]" >&2
    return 1
  fi
elif [ -n "${ZSH_VERSION-}" ]; then
  case "${ZSH_EVAL_CONTEXT-}" in
    *:file)
      echo "ERROR: Do not source this script. Run it: ./script.sh [args]" >&2
      return 1
      ;;
  esac
fi
if [ -z "${BASH_VERSION-}" ]; then
  # Preserve the user's login shell hint for the tmux launcher.
  if [ -z "${OP3_PARENT_SHELL-}" ] && [ -n "${SHELL-}" ]; then
    OP3_PARENT_SHELL="${SHELL##*/}"
    export OP3_PARENT_SHELL
  fi
  if command -v bash >/dev/null 2>&1; then
    exec bash "$0" "$@"
  fi
  echo "ERROR: bash is required to run this script." >&2
  exit 1
fi

set -euo pipefail

# Preserve the user's login shell hint for the tmux launcher even when invoked via shebang.
if [[ -z "${OP3_PARENT_SHELL:-}" && -n "${SHELL:-}" ]]; then
  export OP3_PARENT_SHELL="${SHELL##*/}"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/scripts/op3_tmux/main.sh"

if [[ $# -gt 0 ]]; then
  op3_tmux_main "$@"
  exit $?
fi

if ! command -v gum >/dev/null 2>&1; then
  echo "ERROR: gum is not installed."
  echo "Install: sudo apt install gum"
  exit 1
fi

while true; do
  banner
  choice="$(gum choose --header "OP3 Workspace Menu" \
    "Build workspace" \
    "Health check" \
    "Install deps" \
    "Launch picker" \
    "Docker build" \
    "Docker run" \
    "Docker stop" \
    "Exit tmux session" \
    "Quit")"

  case "$choice" in
    "Build workspace") op3_tmux_main --build ;;
    "Health check") op3_tmux_main --doctor ;;
    "Install deps") op3_tmux_main --install-deps ;;
    "Launch picker")
      op3_tmux_main
      if [[ "$?" -eq 2 ]]; then
        continue
      fi
      exit $?
      ;;
    "Docker build")
      tag="$(gum input --value "${OP3_DOCKER_TAG:-op3_ros2}" --prompt "Image tag: ")"
      build_ws=0
      run_rosdep=1
      gum confirm "Build workspace inside image?" && build_ws=1
      gum confirm "Run rosdep during build?" && run_rosdep=1 || run_rosdep=0
      OP3_DOCKER_TAG="$tag" OP3_DOCKER_BUILD_WS="$build_ws" OP3_DOCKER_RUN_ROSDEP="$run_rosdep" \
        op3_tmux_main --docker-build
      ;;
    "Docker run")
      up_flags=""
      gum confirm "Run in detached mode?" && up_flags="-d"
      gum confirm "Build before run?" && up_flags="--build $up_flags"
      OP3_DOCKER_UP_FLAGS="$up_flags" op3_tmux_main --docker-up
      ;;
    "Docker stop") op3_tmux_main --docker-down ;;
    "Exit tmux session") op3_tmux_main --exit; exit $? ;;
    "Quit"|*) exit 0 ;;
  esac
done
