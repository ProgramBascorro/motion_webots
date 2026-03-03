start_stack() {
  local with_webots="$1"
  local with_manager="$2"
  local with_offset_tuner="$3"
  local with_teleop="$4"
  local with_foxglove="$5"
  local with_tools="$6"
  local with_rqt="$7"
  local with_yolo_vision="$8"
  local with_localization="$9"
  local with_ball_localizer="${10}"
  local with_action_editor="${11}"
  local with_action_web="${12}"
  local with_vision_lab="${13}"
  local with_demo="${14}"
  local dry_run="${15}"

  local needs_ros_stack=0
  if [[ "$with_webots" -eq 1 || "$with_manager" -eq 1 || "$with_offset_tuner" -eq 1 || "$with_teleop" -eq 1 || "$with_foxglove" -eq 1 || "$with_tools" -eq 1 || "$with_rqt" -eq 1 || "$with_yolo_vision" -eq 1 || "$with_localization" -eq 1 || "$with_ball_localizer" -eq 1 || "$with_action_editor" -eq 1 || "$with_action_web" -eq 1 || "$with_demo" -eq 1 ]]; then
    needs_ros_stack=1
  fi

  if [[ "$needs_ros_stack" -eq 1 ]]; then
    health_check "$with_webots"
  else
    ensure_tmux_installed
    auto_detect_shell_runner
  fi
  if [[ "$with_manager" -eq 1 || "$with_action_editor" -eq 1 || "$with_action_web" -eq 1 ]]; then
    ensure_action_file
  fi

  local webots_wrapped manager_wrapped offset_tuner_wrapped teleop_wrapped fox_wrapped tools_wrapped rqt_wrapped
  local yolo_vision_wrapped localization_wrapped ball_localizer_wrapped action_editor_wrapped action_web_wrapped vision_lab_wrapped
  local demo_wrapped demo_cmd
  webots_wrapped="$(wrap_cmd "$WEBOTS_CMD")"
  manager_wrapped="$(wrap_cmd "$MANAGER_CMD")"
  offset_tuner_wrapped="$(wrap_cmd "$OFFSET_TUNER_CMD")"
  teleop_wrapped="$(wrap_cmd "$TELEOP_CMD")"
  fox_wrapped="$(wrap_cmd "$FOXGLOVE_CMD")"
  tools_wrapped="$(wrap_cmd "$TOOLS_CMD")"
  rqt_wrapped="$(wrap_cmd "$RQT_CMD")"
  yolo_vision_wrapped="$(wrap_cmd "$YOLO_VISION_CMD")"
  localization_wrapped="$(wrap_cmd "$LOCALIZATION_CMD")"
  ball_localizer_wrapped="$(wrap_cmd "$BALL_LOCALIZER_CMD")"
  action_editor_wrapped="$(wrap_cmd "$ACTION_EDITOR_CMD")"
  action_web_wrapped="$(wrap_cmd "$ACTION_WEB_CMD")"
  vision_lab_wrapped="$VISION_LAB_CMD"
  demo_cmd="$DEMO_CMD"
  if [[ "$with_demo" -eq 1 && -n "${DEMO_MODE:-}" ]]; then
    local demo_wait_steps="${DEMO_WAIT_STEPS:-40}"
    local demo_wait_step_sec="${DEMO_WAIT_STEP_SEC:-0.5}"
    demo_cmd="${DEMO_CMD} & demo_pid=\$!; for i in \$(seq 1 ${demo_wait_steps}); do if ros2 node list 2>/dev/null | grep -q '^/demo_node$'; then break; fi; sleep ${demo_wait_step_sec}; done; if ros2 node list 2>/dev/null | grep -q '^/demo_node$'; then ros2 topic pub -1 /robotis/mode_command std_msgs/msg/String \\\"{data: ${DEMO_MODE}}\\\"; else echo '[demo] demo_node not found; skipping mode command'; fi; wait \$demo_pid"
  fi
  demo_wrapped="$(wrap_cmd "$demo_cmd")"

  if [[ "$dry_run" -eq 1 ]]; then
    banner
    echo "${DIM}[dry-run] session: $SESSION${RST}"
    echo "${DIM}setup: $SETUP${RST}"
    echo "${DIM}shell: $SHELL_RUNNER${RST}"
    echo
    if [[ "$with_webots" -eq 1 ]]; then echo "${DIM}Pane (webots):  $WEBOTS_CMD${RST}"; fi
    if [[ "$with_manager" -eq 1 ]]; then echo "${DIM}Pane (manager): $MANAGER_CMD${RST}"; fi
    if [[ "$with_offset_tuner" -eq 1 ]]; then echo "${DIM}Pane (offset_tuner): $OFFSET_TUNER_CMD${RST}"; fi
    if [[ "$with_teleop" -eq 1 ]]; then echo "${DIM}Pane (teleop):  $TELEOP_CMD${RST}"; fi
    if [[ "$with_foxglove" -eq 1 ]]; then echo "${DIM}Pane (fox):     $FOXGLOVE_CMD${RST}"; fi
    if [[ "$with_tools" -eq 1 ]]; then echo "${DIM}Pane (tools):   $TOOLS_CMD${RST}"; fi
    if [[ "$with_rqt" -eq 1 ]]; then echo "${DIM}Pane (rqt):     $RQT_CMD${RST}"; fi
    if [[ "$with_yolo_vision" -eq 1 ]]; then echo "${DIM}Pane (yolo_vision):  $YOLO_VISION_CMD${RST}"; fi
    if [[ "$with_localization" -eq 1 ]]; then echo "${DIM}Pane (localization): $LOCALIZATION_CMD${RST}"; fi
    if [[ "$with_ball_localizer" -eq 1 ]]; then echo "${DIM}Pane (ball_localizer): $BALL_LOCALIZER_CMD${RST}"; fi
    if [[ "$with_action_editor" -eq 1 ]]; then echo "${DIM}Pane (action_editor): $ACTION_EDITOR_CMD${RST}"; fi
    if [[ "$with_action_web" -eq 1 ]]; then echo "${DIM}Pane (studio): $ACTION_WEB_CMD${RST}"; fi
    if [[ "$with_vision_lab" -eq 1 ]]; then echo "${DIM}Pane (vision_lab): $VISION_LAB_CMD${RST}"; fi
    if [[ "$with_demo" -eq 1 ]]; then echo "${DIM}Pane (demo): $demo_cmd${RST}"; fi
    echo
    echo "${DIM}Delay between webots->manager: ${START_DELAY_SEC}s${RST}"
    return 0
  fi

  # If session exists, restart it
  if session_exists; then
    echo "${YLW}Session '${SESSION}' exists; restarting...${RST}"
    tmux kill-session -t "$SESSION"
  fi

  # Create base session with ONE window named "main"
  tmux new-session -d -s "$SESSION" -n main

  local components=()
  [[ "$with_webots" -eq 1 ]] && components+=(webots)
  [[ "$with_manager" -eq 1 ]] && components+=(manager)
  [[ "$with_offset_tuner" -eq 1 ]] && components+=(offset_tuner)
  [[ "$with_teleop" -eq 1 ]] && components+=(teleop)
  [[ "$with_foxglove" -eq 1 ]] && components+=(foxglove)
  [[ "$with_tools" -eq 1 ]] && components+=(tools)
  [[ "$with_rqt" -eq 1 ]] && components+=(rqt_image_view)
  [[ "$with_yolo_vision" -eq 1 ]] && components+=(yolo_vision)
  [[ "$with_localization" -eq 1 ]] && components+=(localization)
  [[ "$with_ball_localizer" -eq 1 ]] && components+=(ball_localizer)
  [[ "$with_action_editor" -eq 1 ]] && components+=(action_editor)
  [[ "$with_action_web" -eq 1 ]] && components+=(action_web)
  [[ "$with_vision_lab" -eq 1 ]] && components+=(vision_lab)
  [[ "$with_demo" -eq 1 ]] && components+=(demo)

  local count=${#components[@]}
  if [[ "$count" -lt 1 ]]; then
    die "No components selected."
  fi

  local i
  for ((i=2; i<=count; i++)); do
    if (( i % 2 == 0 )); then
      tmux split-window -h -t "$SESSION":main
    else
      tmux split-window -v -t "$SESSION":main
    fi
    tmux select-layout -t "$SESSION":main tiled
  done

  mapfile -t pane_indices < <(tmux list-panes -t "$SESSION":main -F '#{pane_index}' | sort -n)

  local idx=0
  local pane
  local comp
  for comp in "${components[@]}"; do
    pane="${pane_indices[$idx]}"
    case "$comp" in
      webots)
        tmux_send "$SESSION":main.$pane "$webots_wrapped"
        tmux_env_set "@op3_pane_webots" "$pane"
        ;;
      manager)
        if [[ "$with_webots" -eq 1 ]]; then
          tmux_send "$SESSION":main.$pane "sleep $START_DELAY_SEC; $manager_wrapped"
        else
          tmux_send "$SESSION":main.$pane "$manager_wrapped"
        fi
        tmux_env_set "@op3_pane_manager" "$pane"
        ;;
      offset_tuner)
        tmux_send "$SESSION":main.$pane "$offset_tuner_wrapped"
        tmux_env_set "@op3_pane_offset_tuner" "$pane"
        ;;
      teleop)
        tmux_send "$SESSION":main.$pane "$teleop_wrapped"
        tmux_env_set "@op3_pane_teleop" "$pane"
        ;;
      foxglove)
        tmux_send "$SESSION":main.$pane "$fox_wrapped"
        tmux_env_set "@op3_pane_foxglove" "$pane"
        ;;
      tools)
        tmux_send "$SESSION":main.$pane "$tools_wrapped"
        tmux_env_set "@op3_pane_tools" "$pane"
        ;;
      rqt_image_view)
        tmux_send "$SESSION":main.$pane "$rqt_wrapped"
        tmux_env_set "@op3_pane_rqt" "$pane"
        ;;
      yolo_vision)
        tmux_send "$SESSION":main.$pane "$yolo_vision_wrapped"
        tmux_env_set "@op3_pane_yolo_vision" "$pane"
        ;;
      localization)
        tmux_send "$SESSION":main.$pane "$localization_wrapped"
        tmux_env_set "@op3_pane_localization" "$pane"
        ;;
      ball_localizer)
        tmux_send "$SESSION":main.$pane "$ball_localizer_wrapped"
        tmux_env_set "@op3_pane_ball_localizer" "$pane"
        ;;
      action_editor)
        tmux_send "$SESSION":main.$pane "$action_editor_wrapped"
        tmux_env_set "@op3_pane_action_editor" "$pane"
        ;;
      action_web)
        tmux_send "$SESSION":main.$pane "$action_web_wrapped"
        tmux_env_set "@op3_pane_action_web" "$pane"
        ;;
      vision_lab)
        tmux_send "$SESSION":main.$pane "$vision_lab_wrapped"
        tmux_env_set "@op3_pane_vision_lab" "$pane"
        ;;
      demo)
        tmux_send "$SESSION":main.$pane "$demo_wrapped"
        tmux_env_set "@op3_pane_demo" "$pane"
        ;;
    esac
    idx=$((idx + 1))
  done

  tmux_env_set "@op3_with_teleop" "$with_teleop"
  tmux_env_set "@op3_with_foxglove" "$with_foxglove"
  tmux_env_set "@op3_with_tools" "$with_tools"
  tmux_env_set "@op3_with_rqt" "$with_rqt"
  tmux_env_set "@op3_with_offset_tuner" "$with_offset_tuner"
  tmux_env_set "@op3_with_yolo_vision" "$with_yolo_vision"
  tmux_env_set "@op3_with_localization" "$with_localization"
  tmux_env_set "@op3_with_ball_localizer" "$with_ball_localizer"
  tmux_env_set "@op3_with_action_editor" "$with_action_editor"
  tmux_env_set "@op3_with_action_web" "$with_action_web"
  tmux_env_set "@op3_with_vision_lab" "$with_vision_lab"
  tmux_env_set "@op3_with_demo" "$with_demo"
  tmux_env_set "@op3_profile" "$PROFILE"
  tmux_env_set "@op3_layout" "$count"

  tmux select-pane -t "$SESSION":main.${pane_indices[0]}

  banner
  echo "${GRN}Started.${RST} session '${SESSION}'"
  echo "${BOLD}WS:${RST} $WS"
  echo "${BOLD}SETUP:${RST} $SETUP"
  echo "${BOLD}SHELL:${RST} $SHELL_RUNNER"
  echo "${BOLD}Delay webots→manager:${RST} ${START_DELAY_SEC}s"
  echo "${BOLD}Profile:${RST} $PROFILE"
  echo
  echo "${DIM}Detach: Ctrl+b then d | Kill: ./$(basename "$0") --exit${RST}"
  tmux attach -t "$SESSION"
}
