has_tmux() { command -v tmux >/dev/null 2>&1; }
has_ros2() { command -v ros2 >/dev/null 2>&1; }

ensure_webots_bin() {
  if [[ -n "${WEBOTS_HOME:-}" ]]; then
    [[ -x "$WEBOTS_HOME" ]] || die "WEBOTS_HOME is set but not executable: $WEBOTS_HOME"
    return 0
  fi

  if command -v webots >/dev/null 2>&1; then
    return 0
  fi
  if command -v webots-bin >/dev/null 2>&1; then
    return 0
  fi

  die "Webots binary not found. Install Webots or set WEBOTS_HOME to the executable path."
}

auto_detect_setup() {
  if [[ -n "$SETUP" ]]; then
    [[ -f "$SETUP" ]] || die "Setup file not found: $SETUP"
    return 0
  fi

  local cand1="$WS/install/setup.zsh"
  local cand2="$WS/install/setup.bash"
  local cand3="$WS/install/setup.sh"
  local cand4="$WS/install/local_setup.zsh"
  local cand5="$WS/install/local_setup.bash"

  for c in "$cand1" "$cand2" "$cand3" "$cand4" "$cand5"; do
    if [[ -f "$c" ]]; then
      SETUP="$c"
      return 0
    fi
  done

  die "Could not auto-detect setup file in $WS/install/. Set OP3_SETUP or use --setup PATH."
}

auto_detect_shell_runner() {
  if [[ -n "$SHELL_RUNNER" ]]; then
    return 0
  fi
  case "$SETUP" in
    *.zsh) SHELL_RUNNER="zsh -lc" ;;
    *)     SHELL_RUNNER="bash -lc" ;;
  esac
}

health_check() {
  has_tmux || die "tmux is not installed. Install: sudo apt install tmux"
  has_ros2 || die "ros2 is not installed or not in PATH."

  auto_detect_setup
  auto_detect_shell_runner
  [[ -f "$SETUP" ]] || die "Setup file not found: $SETUP"
  ensure_webots_bin

  if ! $SHELL_RUNNER "source '$SETUP'; ros2 pkg list | grep -q '^op3_joy_teleop$'"; then
    echo "${RED}ERROR:${RST} Workspace is not built/installed (missing op3_joy_teleop)." >&2
    echo "${DIM}Try:${RST} colcon build --packages-select op3_joy_teleop" >&2
    echo "${DIM}Then:${RST} source install/setup.bash | source install/setup.zsh" >&2
    exit 1
  fi
}
