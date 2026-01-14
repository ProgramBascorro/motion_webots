start_stack() {
  local with_webots="$1"
  local with_manager="$2"
  local with_teleop="$3"
  local with_foxglove="$4"
  local with_tools="$5"
  local with_rqt="$6"
  local with_field_vision="$7"
  local with_field_map="$8"
  local with_mcl="$9"
  local dry_run="${10}"

  health_check

  local webots_wrapped manager_wrapped teleop_wrapped fox_wrapped tools_wrapped rqt_wrapped
  local field_vision_wrapped field_map_wrapped mcl_wrapped
  webots_wrapped="$(wrap_cmd "$WEBOTS_CMD")"
  manager_wrapped="$(wrap_cmd "$MANAGER_CMD")"
  teleop_wrapped="$(wrap_cmd "$TELEOP_CMD")"
  fox_wrapped="$(wrap_cmd "$FOXGLOVE_CMD")"
  tools_wrapped="$(wrap_cmd "$TOOLS_CMD")"
  rqt_wrapped="$(wrap_cmd "$RQT_CMD")"
  field_vision_wrapped="$(wrap_cmd "$FIELD_VISION_CMD")"
  field_map_wrapped="$(wrap_cmd "$FIELD_MAP_CMD")"
  mcl_wrapped="$(wrap_cmd "$MCL_CMD")"

  if [[ "$dry_run" -eq 1 ]]; then
    banner
    echo "${DIM}[dry-run] session: $SESSION${RST}"
    echo "${DIM}setup: $SETUP${RST}"
    echo "${DIM}shell: $SHELL_RUNNER${RST}"
    echo
    if [[ "$with_webots" -eq 1 ]]; then echo "${DIM}Pane (webots):  $WEBOTS_CMD${RST}"; fi
    if [[ "$with_manager" -eq 1 ]]; then echo "${DIM}Pane (manager): $MANAGER_CMD${RST}"; fi
    if [[ "$with_teleop" -eq 1 ]]; then echo "${DIM}Pane (teleop):  $TELEOP_CMD${RST}"; fi
    if [[ "$with_foxglove" -eq 1 ]]; then echo "${DIM}Pane (fox):     $FOXGLOVE_CMD${RST}"; fi
    if [[ "$with_tools" -eq 1 ]]; then echo "${DIM}Pane (tools):   $TOOLS_CMD${RST}"; fi
    if [[ "$with_rqt" -eq 1 ]]; then echo "${DIM}Pane (rqt):     $RQT_CMD${RST}"; fi
    if [[ "$with_field_vision" -eq 1 ]]; then echo "${DIM}Pane (field_vision): $FIELD_VISION_CMD${RST}"; fi
    if [[ "$with_field_map" -eq 1 ]]; then echo "${DIM}Pane (field_map):    $FIELD_MAP_CMD${RST}"; fi
    if [[ "$with_mcl" -eq 1 ]]; then echo "${DIM}Pane (mcl):          $MCL_CMD${RST}"; fi
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
  [[ "$with_teleop" -eq 1 ]] && components+=(teleop)
  [[ "$with_foxglove" -eq 1 ]] && components+=(foxglove)
  [[ "$with_tools" -eq 1 ]] && components+=(tools)
  [[ "$with_rqt" -eq 1 ]] && components+=(rqt_image_view)
  [[ "$with_field_vision" -eq 1 ]] && components+=(field_vision)
  [[ "$with_field_map" -eq 1 ]] && components+=(field_map)
  [[ "$with_mcl" -eq 1 ]] && components+=(mcl)

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
    tmux send-keys -t "$SESSION":main.$pane "$SHELL_RUNNER" C-m
    case "$comp" in
      webots)
        tmux send-keys -t "$SESSION":main.$pane "$webots_wrapped" C-m
        tmux_env_set "@op3_pane_webots" "$pane"
        ;;
      manager)
        if [[ "$with_webots" -eq 1 ]]; then
          tmux send-keys -t "$SESSION":main.$pane "sleep $START_DELAY_SEC; $manager_wrapped" C-m
        else
          tmux send-keys -t "$SESSION":main.$pane "$manager_wrapped" C-m
        fi
        tmux_env_set "@op3_pane_manager" "$pane"
        ;;
      teleop)
        tmux send-keys -t "$SESSION":main.$pane "$teleop_wrapped" C-m
        tmux_env_set "@op3_pane_teleop" "$pane"
        ;;
      foxglove)
        tmux send-keys -t "$SESSION":main.$pane "$fox_wrapped" C-m
        tmux_env_set "@op3_pane_foxglove" "$pane"
        ;;
      tools)
        tmux send-keys -t "$SESSION":main.$pane "$tools_wrapped" C-m
        tmux_env_set "@op3_pane_tools" "$pane"
        ;;
      rqt_image_view)
        tmux send-keys -t "$SESSION":main.$pane "$rqt_wrapped" C-m
        tmux_env_set "@op3_pane_rqt" "$pane"
        ;;
      field_vision)
        tmux send-keys -t "$SESSION":main.$pane "$field_vision_wrapped" C-m
        tmux_env_set "@op3_pane_field_vision" "$pane"
        ;;
      field_map)
        tmux send-keys -t "$SESSION":main.$pane "$field_map_wrapped" C-m
        tmux_env_set "@op3_pane_field_map" "$pane"
        ;;
      mcl)
        tmux send-keys -t "$SESSION":main.$pane "$mcl_wrapped" C-m
        tmux_env_set "@op3_pane_mcl" "$pane"
        ;;
    esac
    idx=$((idx + 1))
  done

  tmux_env_set "@op3_with_teleop" "$with_teleop"
  tmux_env_set "@op3_with_foxglove" "$with_foxglove"
  tmux_env_set "@op3_with_tools" "$with_tools"
  tmux_env_set "@op3_with_rqt" "$with_rqt"
  tmux_env_set "@op3_with_field_vision" "$with_field_vision"
  tmux_env_set "@op3_with_field_map" "$with_field_map"
  tmux_env_set "@op3_with_mcl" "$with_mcl"
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
