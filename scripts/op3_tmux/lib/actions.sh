action_exit() {
  if session_exists; then
    echo "${YLW}Stopping session${RST} ${BOLD}$SESSION${RST} ..."
    tmux kill-session -t "$SESSION"
    echo "${GRN}Done.${RST}"
  else
    echo "${DIM}Session '$SESSION' is not running.${RST}"
  fi
}

action_status() {
  if session_exists; then
    echo "${GRN}Running:${RST} tmux session '${SESSION}' exists."
    tmux list-windows -t "$SESSION" || true
  else
    echo "${YLW}Not running:${RST} tmux session '${SESSION}' not found."
  fi
}

action_doctor() {
  doctor_check
}

action_install_deps() {
  local apt_cmd="apt-get"
  if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
    apt_cmd="sudo apt-get"
  fi

  if ! command -v apt-get >/dev/null 2>&1; then
    die "apt-get not found; this command supports Debian/Ubuntu only."
  fi

  local base_pkgs=(
    build-essential
    cmake
    git
    python3-colcon-common-extensions
    python3-rosdep
  )
  local ros_pkgs=(
    ros-humble-rosbridge-server
    ros-humble-octomap-ros
    ros-humble-octomap-server
    ros-humble-octomap-msgs
    ros-humble-joy
    ros-humble-foxglove-bridge
    ros-humble-rqt-image-view
  )

  echo "${GRN}Installing base packages...${RST}"
  $apt_cmd update
  $apt_cmd install -y --no-install-recommends "${base_pkgs[@]}" "${ros_pkgs[@]}"

  if command -v rosdep >/dev/null 2>&1; then
    if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
      sudo rosdep init 2>/dev/null || true
    else
      rosdep init 2>/dev/null || true
    fi
    rosdep update
    if [[ -d "$WS/src" ]]; then
      (cd "$WS" && rosdep install --from-paths src --ignore-src -r -y)
    else
      echo "${YLW}Warning:${RST} $WS/src not found; skip rosdep install."
    fi
  else
    echo "${YLW}Warning:${RST} rosdep not found after install."
  fi

  echo "${GRN}Done:${RST} dependencies installed."
}

action_build() {
  auto_detect_shell_runner
  if [[ ! -d "$WS" ]]; then
    die "Workspace not found: $WS"
  fi
  local ros_setup="/opt/ros/humble/setup.bash"
  if [[ "$SHELL_RUNNER" == zsh* ]] && [[ -f "/opt/ros/humble/setup.zsh" ]]; then
    ros_setup="/opt/ros/humble/setup.zsh"
  fi
  local cmd="cd '$WS'; source '$ros_setup'; colcon build --merge-install --symlink-install"
  run_in_shell "$cmd"
}

action_attach() {
  if session_exists; then
    tmux attach -t "$SESSION"
  else
    die "Session '$SESSION' is not running."
  fi
}

action_stop() {
  if ! session_exists; then
    echo "${DIM}Session '$SESSION' is not running.${RST}"
    return 0
  fi

  auto_detect_setup
  auto_detect_shell_runner

  local stop_cmd
  stop_cmd="source '$SETUP'; ros2 topic pub --once /robotis/walking/command std_msgs/msg/String \"{data: 'stop'}\""
  run_in_shell "$stop_cmd" || true

  local topic_type
  topic_type="$(run_in_shell "source '$SETUP'; ros2 topic info -v /robotis/walking/set_params" | awk -F': ' '/^ *Type:/ {print $2; exit}')"
  if [[ -n "$topic_type" ]]; then
    local zero_msg
    zero_msg="{x_move_amplitude: 0.0, y_move_amplitude: 0.0, angle_move_amplitude: 0.0}"
    run_in_shell "source '$SETUP'; ros2 topic pub --once /robotis/walking/set_params $topic_type '$zero_msg'" || true
  else
    echo "${YLW}Warning:${RST} Could not detect /robotis/walking/set_params type; skipped zero params."
  fi

  tmux kill-session -t "$SESSION"
  echo "${GRN}Sent stop + zero, then killed session.${RST}"
}

docker_base_cmd() {
  if ! command -v docker >/dev/null 2>&1; then
    die "docker not found. Install Docker first."
  fi
  if groups "$USER" 2>/dev/null | grep -q "\bdocker\b"; then
    echo "docker"
    return 0
  fi
  if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
    echo "docker"
    return 0
  fi
  echo "sudo docker"
}

docker_compose_cmd() {
  local docker_cmd
  docker_cmd="$(docker_base_cmd)"
  if $docker_cmd compose version >/dev/null 2>&1; then
    echo "$docker_cmd compose"
    return 0
  fi
  if command -v docker-compose >/dev/null 2>&1; then
    if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
      echo "docker-compose"
    else
      echo "sudo docker-compose"
    fi
    return 0
  fi
  die "docker compose not found. Install docker or docker-compose."
}

action_docker_build() {
  local docker_cmd
  docker_cmd="$(docker_base_cmd)"
  [[ -d "$WS" ]] || die "Workspace not found: $WS"

  local tag="${OP3_DOCKER_TAG:-op3-webots-ros2:humble}"
  local with_webots="${OP3_DOCKER_WITH_WEBOTS:-1}"
  local webots_version="${OP3_DOCKER_WEBOTS_VERSION:-R2025a}"
  local webots_pkg_prefix="${OP3_DOCKER_WEBOTS_PACKAGE_PREFIX:-}"
  local build_flags="${OP3_DOCKER_BUILD_FLAGS:-}"

  (cd "$WS" && $docker_cmd build \
    -t "$tag" \
    --build-arg WITH_WEBOTS="$with_webots" \
    --build-arg WEBOTS_VERSION="$webots_version" \
    --build-arg WEBOTS_PACKAGE_PREFIX="$webots_pkg_prefix" \
    $build_flags \
    .)
}

action_docker_run() {
  local docker_cmd
  docker_cmd="$(docker_base_cmd)"
  [[ -d "$WS" ]] || die "Workspace not found: $WS"

  local tag="${OP3_DOCKER_TAG:-op3-webots-ros2:humble}"
  local run_flags="${OP3_DOCKER_RUN_FLAGS:-}"
  local xauth="${XAUTHORITY:-$HOME/.Xauthority}"

  local -a docker_cmd_parts
  read -r -a docker_cmd_parts <<< "$docker_cmd"

  local -a cmd
  cmd=(
    "${docker_cmd_parts[@]}" run -it --rm
    --net=host --ipc=host
    --privileged
    --ulimit rtprio=99
    --ulimit memlock=-1
    --cap-add SYS_NICE
    --cap-add SYS_RESOURCE
  )

  [[ -e /dev/ttyUSB0 ]] && cmd+=(--device=/dev/ttyUSB0)
  [[ -e /dev/input ]] && cmd+=(--device=/dev/input)
  [[ -e /dev/uinput ]] && cmd+=(--device=/dev/uinput)

  local input_gid=""
  local dialout_gid=""
  input_gid="$(getent group input 2>/dev/null | cut -d: -f3 || true)"
  dialout_gid="$(getent group dialout 2>/dev/null | cut -d: -f3 || true)"
  [[ -n "$input_gid" ]] && cmd+=(--group-add "$input_gid")
  [[ -n "$dialout_gid" ]] && cmd+=(--group-add "$dialout_gid")

  if [[ -n "${DISPLAY:-}" ]]; then
    cmd+=(-e "DISPLAY=${DISPLAY}")
    [[ -S /tmp/.X11-unix/X0 ]] && cmd+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    [[ -f "$xauth" ]] && cmd+=(-e XAUTHORITY=/tmp/.Xauthority -v "$xauth":/tmp/.Xauthority:rw)
  fi

  cmd+=(-v "$WS":/ros2_ws -w /ros2_ws)

  if [[ -n "$run_flags" ]]; then
    read -r -a extra_flags <<< "$run_flags"
    cmd+=("${extra_flags[@]}")
  fi

  cmd+=("$tag" bash)
  "${cmd[@]}"
}

action_docker_up() {
  local compose_cmd
  compose_cmd="$(docker_compose_cmd)"
  [[ -d "$WS" ]] || die "Workspace not found: $WS"
  local up_flags="${OP3_DOCKER_UP_FLAGS:-}"
  (cd "$WS" && $compose_cmd up $up_flags)
}

action_docker_down() {
  local compose_cmd
  compose_cmd="$(docker_compose_cmd)"
  [[ -d "$WS" ]] || die "Workspace not found: $WS"
  (cd "$WS" && $compose_cmd down)
}

restart_component() {
  local comp="$1"

  session_exists || die "Session '$SESSION' is not running."
  local require_webots=0
  [[ "$comp" == "webots" ]] && require_webots=1
  health_check "$require_webots"

  local pane=""
  local target=""

  case "$comp" in
    webots) pane="$(tmux_env_get @op3_pane_webots)" ;;
    manager) pane="$(tmux_env_get @op3_pane_manager)" ;;
    teleop) pane="$(tmux_env_get @op3_pane_teleop)" ;;
    foxglove) pane="$(tmux_env_get @op3_pane_foxglove)" ;;
    tools) pane="$(tmux_env_get @op3_pane_tools)" ;;
    rqt_image_view|rqt) pane="$(tmux_env_get @op3_pane_rqt)" ;;
    yolo_vision) pane="$(tmux_env_get @op3_pane_yolo_vision)" ;;
    localization) pane="$(tmux_env_get @op3_pane_localization)" ;;
    ball_localizer) pane="$(tmux_env_get @op3_pane_ball_localizer)" ;;
    action_editor) pane="$(tmux_env_get @op3_pane_action_editor)" ;;
    action_web) pane="$(tmux_env_get @op3_pane_action_web)" ;;
    demo) pane="$(tmux_env_get @op3_pane_demo)" ;;
    *) die "Unknown component for --restart: $comp" ;;
  esac

  if [[ -z "$pane" ]]; then
    echo "${DIM}Component '$comp' was not started.${RST}"
    return 0
  fi

  target="$SESSION:main.$pane"

  tmux send-keys -t "$target" C-c
  sleep 0.5

  local wrapped=""
  case "$comp" in
    webots) wrapped="$(wrap_cmd "$WEBOTS_CMD")" ;;
    manager) wrapped="$(wrap_cmd "$MANAGER_CMD")" ;;
    teleop) wrapped="$(wrap_cmd "$TELEOP_CMD")" ;;
    foxglove) wrapped="$(wrap_cmd "$FOXGLOVE_CMD")" ;;
    tools) wrapped="$(wrap_cmd "$TOOLS_CMD")" ;;
    rqt_image_view|rqt) wrapped="$(wrap_cmd "$RQT_CMD")" ;;
    yolo_vision) wrapped="$(wrap_cmd "$YOLO_VISION_CMD")" ;;
    localization) wrapped="$(wrap_cmd "$LOCALIZATION_CMD")" ;;
    ball_localizer) wrapped="$(wrap_cmd "$BALL_LOCALIZER_CMD")" ;;
    action_editor) wrapped="$(wrap_cmd "$ACTION_EDITOR_CMD")" ;;
    action_web) wrapped="$(wrap_cmd "$ACTION_WEB_CMD")" ;;
    demo) wrapped="$(wrap_cmd "$DEMO_CMD")" ;;
  esac

  tmux_send "$target" "$wrapped"
  echo "${GRN}Restarted:${RST} $comp"
}
