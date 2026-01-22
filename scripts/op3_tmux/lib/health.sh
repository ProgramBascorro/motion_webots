has_tmux() { command -v tmux >/dev/null 2>&1; }
has_ros2() { command -v ros2 >/dev/null 2>&1; }
has_cmd() { command -v "$1" >/dev/null 2>&1; }

webots_available() {
  if [[ -n "${WEBOTS_HOME:-}" ]]; then
    [[ -d "$WEBOTS_HOME" ]] && return 0
    [[ -x "$WEBOTS_HOME" ]] && return 0
  fi
  command -v webots >/dev/null 2>&1 && return 0
  command -v webots-bin >/dev/null 2>&1 && return 0
  return 1
}

detect_setup_optional() {
  if [[ -n "${SETUP:-}" ]]; then
    [[ -f "$SETUP" ]] && return 0
    return 1
  fi

  # Prefer bash/sh setup scripts since the project entrypoint is bash. Use zsh
  # only if explicitly requested via --setup/OP3_SETUP or OP3_SHELL_RUNNER.
  local cand1="$WS/install/setup.bash"
  local cand2="$WS/install/setup.sh"
  local cand3="$WS/install/setup.zsh"
  local cand4="$WS/install/local_setup.bash"
  local cand5="$WS/install/local_setup.sh"
  local cand6="$WS/install/local_setup.zsh"

  for c in "$cand1" "$cand2" "$cand3" "$cand4" "$cand5" "$cand6"; do
    if [[ -f "$c" ]]; then
      SETUP="$c"
      return 0
    fi
  done
  return 1
}

check_ros_pkg() {
  local pkg="$1"
  if [[ -z "${SETUP:-}" ]]; then
    return 1
  fi
  $SHELL_RUNNER "source '$SETUP'; ros2 pkg prefix $pkg >/dev/null 2>&1"
}

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

  # Prefer bash/sh setup scripts since the project entrypoint is bash.
  local cand1="$WS/install/setup.bash"
  local cand2="$WS/install/setup.sh"
  local cand3="$WS/install/setup.zsh"
  local cand4="$WS/install/local_setup.bash"
  local cand5="$WS/install/local_setup.sh"
  local cand6="$WS/install/local_setup.zsh"

  for c in "$cand1" "$cand2" "$cand3" "$cand4" "$cand5" "$cand6"; do
    if [[ -f "$c" ]]; then
      SETUP="$c"
      return 0
    fi
  done

  die "Could not auto-detect setup file in $WS/install/. Set OP3_SETUP or use --setup PATH."
}

auto_detect_shell_runner() {
  if [[ -n "$SHELL_RUNNER" ]]; then
    # If the user forces zsh, fail fast with a clearer message.
    if [[ "$SHELL_RUNNER" == zsh* ]] && ! command -v zsh >/dev/null 2>&1; then
      die "OP3_SHELL_RUNNER is set to zsh but zsh is not installed. Install zsh or set OP3_SHELL_RUNNER='bash -lc'."
    fi
    return 0
  fi

  if [[ "$SETUP" == *.zsh ]]; then
    if command -v zsh >/dev/null 2>&1; then
      SHELL_RUNNER="zsh -lc"
      return 0
    fi

    # zsh isn't available; fall back to a bash/sh setup script if present.
    local alt_bash="${SETUP%.zsh}.bash"
    local alt_sh="${SETUP%.zsh}.sh"
    if [[ -f "$alt_bash" ]]; then
      SETUP="$alt_bash"
      SHELL_RUNNER="bash -lc"
      return 0
    fi
    if [[ -f "$alt_sh" ]]; then
      SETUP="$alt_sh"
      SHELL_RUNNER="bash -lc"
      return 0
    fi

    die "Setup script is zsh-only ($SETUP) but zsh is not installed. Use --setup '$WS/install/setup.bash' or install zsh."
  fi

  SHELL_RUNNER="bash -lc"
}

health_check() {
  has_tmux || die "tmux is not installed. Install: sudo apt install tmux"
  has_ros2 || die "ros2 is not installed or not in PATH."

  auto_detect_setup
  auto_detect_shell_runner
  [[ -f "$SETUP" ]] || die "Setup file not found: $SETUP"
  ensure_webots_bin

  if ! $SHELL_RUNNER "source '$SETUP'; ros2 pkg prefix op3_joy_teleop >/dev/null 2>&1"; then
    echo "${RED}ERROR:${RST} Workspace is not built/installed (missing op3_joy_teleop)." >&2
    echo "${DIM}Try:${RST} colcon build --packages-select op3_joy_teleop" >&2
    echo "${DIM}Then:${RST} source install/setup.bash" >&2
    exit 1
  fi
}

doctor_check() {
  local ok=1

  if has_tmux; then
    echo "${GRN}OK:${RST} tmux"
  else
    echo "${YLW}WARN:${RST} tmux not found"
    ok=0
  fi

  if has_ros2; then
    echo "${GRN}OK:${RST} ros2"
  else
    echo "${YLW}WARN:${RST} ros2 not found in PATH"
    ok=0
  fi

  if has_cmd colcon; then
    echo "${GRN}OK:${RST} colcon"
  else
    echo "${YLW}WARN:${RST} colcon not found (install python3-colcon-common-extensions)"
    ok=0
  fi

  if has_cmd rosdep; then
    echo "${GRN}OK:${RST} rosdep"
  else
    echo "${YLW}WARN:${RST} rosdep not found (install python3-rosdep)"
    ok=0
  fi

  if has_cmd gum; then
    echo "${GRN}OK:${RST} gum"
  else
    echo "${YLW}WARN:${RST} gum not found (install gum for menus)"
    ok=0
  fi

  if has_cmd docker; then
    echo "${GRN}OK:${RST} docker"
  else
    echo "${YLW}WARN:${RST} docker not found (install Docker for container workflows)"
  fi

  if webots_available; then
    echo "${GRN}OK:${RST} Webots"
  else
    echo "${YLW}WARN:${RST} Webots not found (install or set WEBOTS_HOME)"
    ok=0
  fi

  if detect_setup_optional; then
    auto_detect_shell_runner
    echo "${GRN}OK:${RST} setup: $SETUP"
    if $SHELL_RUNNER "source '$SETUP'; ros2 pkg prefix op3_joy_teleop >/dev/null 2>&1"; then
      echo "${GRN}OK:${RST} workspace install (op3_joy_teleop)"
    else
      echo "${YLW}WARN:${RST} workspace not built (missing op3_joy_teleop)"
      ok=0
    fi

    local ros_pkgs=(
      rosbridge_server
      octomap_ros
      octomap_msgs
      octomap_server
      webots_ros2_driver
      foxglove_bridge
      joy
    )
    for pkg in "${ros_pkgs[@]}"; do
      if check_ros_pkg "$pkg"; then
        echo "${GRN}OK:${RST} ros2 pkg $pkg"
      else
        echo "${YLW}WARN:${RST} ros2 pkg $pkg missing"
        ok=0
      fi
    done
  else
    echo "${YLW}WARN:${RST} setup not found in $WS/install (build first)"
    ok=0
  fi

  if [[ "$ok" -eq 1 ]]; then
    echo "${GRN}Doctor:${RST} all checks passed."
    return 0
  fi

  echo "${YLW}Doctor:${RST} issues found. Run --install-deps or build."
  return 1
}
