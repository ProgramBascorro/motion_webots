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

# -----------------------------
# Safe enum-style menu handling
# -----------------------------

# Canonical enum IDs (stable, safe for case/esac)
readonly M_LAUNCH_PICKER="launch_picker"
readonly M_BUILD="build"
readonly M_DOCTOR="doctor"
readonly M_INSTALL_DEPS="install_deps"
readonly M_DOCKER_BUILD="docker_build"
readonly M_DOCKER_RUN="docker_run"
readonly M_DOCKER_COMPOSE_UP="docker_compose_up"
readonly M_DOCKER_STOP="docker_stop"
readonly M_EXIT_TMUX="exit_tmux"
readonly M_QUIT="quit"

# Order matters (Launch picker first)
readonly MENU_ENUMS=(
  "$M_LAUNCH_PICKER"
  "$M_BUILD"
  "$M_DOCTOR"
  "$M_INSTALL_DEPS"
  "$M_DOCKER_BUILD"
  "$M_DOCKER_RUN"
  "$M_DOCKER_COMPOSE_UP"
  "$M_DOCKER_STOP"
  "$M_EXIT_TMUX"
  "$M_QUIT"
)

# Pretty label (no numbering here)
menu_label() {
  case "$1" in
    "$M_LAUNCH_PICKER") echo "Launch picker" ;;
    "$M_BUILD")         echo "Build workspace" ;;
    "$M_DOCTOR")        echo "Health check" ;;
    "$M_INSTALL_DEPS")  echo "Install deps" ;;
    "$M_DOCKER_BUILD")  echo "Docker build" ;;
    "$M_DOCKER_RUN")    echo "Docker run (container)" ;;
    "$M_DOCKER_COMPOSE_UP") echo "Docker compose up" ;;
    "$M_DOCKER_STOP")   echo "Docker compose down" ;;
    "$M_EXIT_TMUX")     echo "Exit tmux session" ;;
    "$M_QUIT")          echo "Quit" ;;
    *)                  echo "Unknown" ;;
  esac
}

# Emoji per enum (ASCII-safe fallback: leave empty if you want)
menu_emoji() {
  case "$1" in
    "$M_LAUNCH_PICKER") echo "🚀" ;;
    "$M_BUILD")         echo "🔧" ;;
    "$M_DOCTOR")        echo "🩺" ;;
    "$M_INSTALL_DEPS")  echo "📦" ;;
    "$M_DOCKER_BUILD")  echo "🏗️" ;;
    "$M_DOCKER_RUN")    echo "🐳" ;;
    "$M_DOCKER_COMPOSE_UP") echo "🐳" ;;
    "$M_DOCKER_STOP")   echo "🛑" ;;
    "$M_EXIT_TMUX")     echo "🧹" ;;
    "$M_QUIT")          echo "👋" ;;
    *)                  echo "" ;;
  esac
}

# Build the exact display string for gum (numbered + emoji + label)
menu_item() {
  local idx="$1"
  local id="$2"
  local emoji label
  emoji="$(menu_emoji "$id")"
  label="$(menu_label "$id")"
  printf '%2d) %s %s' "$idx" "$emoji" "$label"
}

# Convert gum selection -> enum id (robust against numbering/emojis/spacing)
choice_to_enum() {
  local picked="$1"
  # Strip leading number bullets like " 1) " / "01.) " etc
  picked="$(printf '%s' "$picked" | sed -E 's/^[[:space:]]*[0-9]+[[:space:]]*[\)\.:-]*[[:space:]]*//')"
  # Remove leading emoji-ish token (best-effort): first "word" if it contains non-alnum
  # (Keeps it simple; labels are the real source of truth.)
  picked="$(printf '%s' "$picked" | sed -E 's/^[^[:alnum:]]+[[:space:]]+//')"

  case "$picked" in
    "Launch picker")     echo "$M_LAUNCH_PICKER" ;;
    "Build workspace")   echo "$M_BUILD" ;;
    "Health check")      echo "$M_DOCTOR" ;;
    "Install deps")      echo "$M_INSTALL_DEPS" ;;
    "Docker build")      echo "$M_DOCKER_BUILD" ;;
    "Docker run (container)") echo "$M_DOCKER_RUN" ;;
    "Docker compose up") echo "$M_DOCKER_COMPOSE_UP" ;;
    "Docker compose down") echo "$M_DOCKER_STOP" ;;
    "Exit tmux session") echo "$M_EXIT_TMUX" ;;
    "Quit")              echo "$M_QUIT" ;;
    *)                   echo "$M_QUIT" ;;
  esac
}

# If args provided, pass through (unchanged behavior)
if [[ $# -gt 0 ]]; then
  op3_tmux_main "$@"
  exit $?
fi

if ! command -v gum >/dev/null 2>&1; then
  echo "gum is not installed."
  cat <<'EOF'
Install with:
  sudo mkdir -p /etc/apt/keyrings
  curl -fsSL https://repo.charm.sh/apt/gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/charm.gpg
  echo "deb [signed-by=/etc/apt/keyrings/charm.gpg] https://repo.charm.sh/apt/ * *" | sudo tee /etc/apt/sources.list.d/charm.list
  sudo apt update && sudo apt install gum
EOF
  if [[ -t 0 && -t 1 ]]; then
    read -r -p "Install gum now? [y/N] " reply
    case "$reply" in
      y|Y|yes|YES)
        sudo mkdir -p /etc/apt/keyrings
        curl -fsSL https://repo.charm.sh/apt/gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/charm.gpg
        echo "deb [signed-by=/etc/apt/keyrings/charm.gpg] https://repo.charm.sh/apt/ * *" | sudo tee /etc/apt/sources.list.d/charm.list
        sudo apt update && sudo apt install gum
        ;;
    esac
  fi
  command -v gum >/dev/null 2>&1 || exit 1
fi

# Build numbered menu items (single source of truth = MENU_ENUMS)
menu_items=()
i=1
for id in "${MENU_ENUMS[@]}"; do
  menu_items+=("$(menu_item "$i" "$id")")
  i=$((i + 1))
done

while true; do
  banner

  picked="$(gum choose --header "OP3 Workspace Menu" "${menu_items[@]}")"
  action="$(choice_to_enum "$picked")"

  case "$action" in
    "$M_LAUNCH_PICKER")
      op3_tmux_main
      rc=$?
      if [[ $rc -eq 2 ]]; then
        continue
      fi
      exit $rc
      ;;

    "$M_BUILD")
      op3_tmux_main --build
      ;;

    "$M_DOCTOR")
      op3_tmux_main --doctor
      ;;

    "$M_INSTALL_DEPS")
      op3_tmux_main --install-deps
      ;;

    "$M_DOCKER_BUILD")
      tag="$(gum input --value "${OP3_DOCKER_TAG:-op3_ros2}" --prompt "Image tag: ")"
      build_ws=0
      run_rosdep=1
      gum confirm "Build workspace inside image?" && build_ws=1
      gum confirm "Run rosdep during build?" && run_rosdep=1 || run_rosdep=0
      OP3_DOCKER_TAG="$tag" OP3_DOCKER_BUILD_WS="$build_ws" OP3_DOCKER_RUN_ROSDEP="$run_rosdep" \
        op3_tmux_main --docker-build
      ;;

    "$M_DOCKER_RUN")
      run_flags=""
      gum confirm "Run in detached mode?" && run_flags="-d"
      OP3_DOCKER_RUN_FLAGS="$run_flags" op3_tmux_main --docker-run
      ;;

    "$M_DOCKER_COMPOSE_UP")
      up_flags=""
      gum confirm "Run in detached mode?" && up_flags="-d"
      gum confirm "Build before run?" && up_flags="--build $up_flags"
      OP3_DOCKER_UP_FLAGS="$up_flags" op3_tmux_main --docker-up
      ;;

    "$M_DOCKER_STOP")
      op3_tmux_main --docker-down
      ;;

    "$M_EXIT_TMUX")
      op3_tmux_main --exit
      exit $?
      ;;

    "$M_QUIT"|*)
      exit 0
      ;;
  esac
done
