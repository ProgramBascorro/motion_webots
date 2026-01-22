#!/usr/bin/env bash
set -euo pipefail

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
