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

restart_component() {
  local comp="$1"

  session_exists || die "Session '$SESSION' is not running."
  health_check

  local pane=""
  local target=""

  case "$comp" in
    webots) pane="$(tmux_env_get @op3_pane_webots)" ;;
    manager) pane="$(tmux_env_get @op3_pane_manager)" ;;
    teleop) pane="$(tmux_env_get @op3_pane_teleop)" ;;
    foxglove) pane="$(tmux_env_get @op3_pane_foxglove)" ;;
    tools) pane="$(tmux_env_get @op3_pane_tools)" ;;
    rqt_image_view|rqt) pane="$(tmux_env_get @op3_pane_rqt)" ;;
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
  esac

  tmux_send "$target" "$wrapped"
  echo "${GRN}Restarted:${RST} $comp"
}
