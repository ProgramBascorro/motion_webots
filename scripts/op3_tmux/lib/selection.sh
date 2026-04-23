COMPONENTS=(webots manager_sim manager_real offset_tuner demo teleop action_web vision_lab rqt_image_view yolo_vision localization ball_head_tracking ball_localizer action_editor foxglove tools)
MENU_ACTION=""
SELECTION_CANCELLED="__CANCEL__"
MANAGER_MODE=""
MANAGER_CONFLICT=0

demo_launches_manager() {
  [[ "${DEMO_CMD:-}" != *"with_manager:=false"* ]]
}

component_label() {
  case "$1" in
    webots) echo "Webots" ;;
    manager_sim) echo "Manager (Sim)" ;;
    manager_real) echo "Manager (Real)" ;;
    offset_tuner) echo "Offset Tuner" ;;
    demo) echo "Demo" ;;
    teleop) echo "Teleop" ;;
    action_web) echo "Studio" ;;
    vision_lab) echo "Vision Lab" ;;
    rqt_image_view) echo "RQT Image View" ;;
    yolo_vision) echo "YOLO Vision" ;;
    localization) echo "Localization" ;;
    ball_head_tracking) echo "Ball Head Tracking" ;;
    ball_localizer) echo "Ball Localizer" ;;
    action_editor) echo "Action Editor" ;;
    foxglove) echo "Foxglove" ;;
    tools) echo "Tools" ;;
    *) echo "$1" ;;
  esac
}

component_id() {
  case "$1" in
    "Webots") echo "webots" ;;
    "Manager (Sim)") echo "manager_sim" ;;
    "Manager (Real)") echo "manager_real" ;;
    "Offset Tuner") echo "offset_tuner" ;;
    "Demo") echo "demo" ;;
    "Teleop") echo "teleop" ;;
    "Studio") echo "action_web" ;;
    "Vision Lab") echo "vision_lab" ;;
    "RQT Image View") echo "rqt_image_view" ;;
    "YOLO Vision") echo "yolo_vision" ;;
    "Localization") echo "localization" ;;
    "Ball Head Tracking") echo "ball_head_tracking" ;;
    "Ball Localizer") echo "ball_localizer" ;;
    "Action Editor") echo "action_editor" ;;
    "Foxglove") echo "foxglove" ;;
    "Tools") echo "tools" ;;
    *) echo "$1" ;;
  esac
}

usual_file_path() {
  echo "${OP3_CONFIG_DIR:-${HOME}/.config/op3-stack}/usual.txt"
}

last_file_path() {
  echo "${OP3_CONFIG_DIR:-${HOME}/.config/op3-stack}/last.txt"
}

load_usual_selection() {
  local file
  file="$(usual_file_path)"
  if [[ -f "$file" ]]; then
    grep -v '^[[:space:]]*$' "$file"
  else
    printf "%s\n" manager_sim
  fi
}

load_last_selection() {
  local file
  file="$(last_file_path)"
  if [[ -f "$file" ]]; then
    grep -v '^[[:space:]]*$' "$file"
  else
    return 1
  fi
}

save_usual_selection() {
  local file
  file="$(usual_file_path)"
  mkdir -p "$(dirname "$file")" 2>/dev/null || return 0
  printf "%s\n" "$@" > "$file" 2>/dev/null || true
}

save_last_selection() {
  local file
  file="$(last_file_path)"
  mkdir -p "$(dirname "$file")" 2>/dev/null || return 0
  printf "%s\n" "$@" > "$file" 2>/dev/null || true
}

join_by_comma() {
  local IFS=,
  echo "$*"
}

select_with_gum() {
  local selected_csv=""
  local picked=""
  local -a labels=()
  local -a selected_labels=()
  if [[ -f "$(last_file_path)" ]]; then
    mapfile -t _last < <(load_last_selection)
    if [[ ${#_last[@]} -gt 0 ]]; then
      for item in "${_last[@]}"; do
        selected_labels+=("$(component_label "$item")")
      done
      selected_csv="$(join_by_comma "${selected_labels[@]}")"
    fi
  elif [[ -f "$(usual_file_path)" ]]; then
    mapfile -t _usual < <(load_usual_selection)
    if [[ ${#_usual[@]} -gt 0 ]]; then
      for item in "${_usual[@]}"; do
        selected_labels+=("$(component_label "$item")")
      done
      selected_csv="$(join_by_comma "${selected_labels[@]}")"
    fi
  fi
  [[ -z "$selected_csv" ]] && selected_csv=""

  for item in "${COMPONENTS[@]}"; do
    labels+=("$(component_label "$item")")
  done

  if [[ -n "$selected_csv" ]]; then
    local -a filtered=()
    local label
    for label in "${selected_labels[@]}"; do
      [[ -z "$label" ]] && continue
      if printf '%s\n' "${labels[@]}" | grep -Fxq -- "$label"; then
        filtered+=("$label")
      fi
    done
    selected_csv=""
    if [[ ${#filtered[@]} -gt 0 ]]; then
      selected_csv="$(join_by_comma "${filtered[@]}")"
    fi
  fi

  if [[ -n "$selected_csv" ]]; then
    picked="$(gum choose --no-limit --selected "$selected_csv" --header "Use <space> to toggle, <enter> to start" "${labels[@]}")" || {
      printf '%s\n' "$SELECTION_CANCELLED"
      return 0
    }
  else
    picked="$(gum choose --no-limit --header "Use <space>  to toggle, <enter> to start" "${labels[@]}")" || {
      printf '%s\n' "$SELECTION_CANCELLED"
      return 0
    }
  fi
  if [[ -z "$picked" ]]; then
    printf '%s\n' "$SELECTION_CANCELLED"
    return 0
  fi
  while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    printf '%s\n' "$(component_id "$line")"
  done <<< "$picked"
  return 0
}

interactive_selection() {
  # banner >&2
  if ! command -v gum >/dev/null 2>&1; then
    echo "${YLW}gum is not installed.${RST}" >&2
    cat <<'EOF' >&2
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
  select_with_gum
}

validate_selection() {
  local has_manager=0
  local has_teleop=0
  local has_foxglove=0
  local has_demo=0

  local item
  for item in "$@"; do
    case "$item" in
      manager|manager_sim|manager_real) has_manager=1 ;;
      demo) has_demo=1 ;;
      teleop) has_teleop=1 ;;
      foxglove) has_foxglove=1 ;;
    esac
  done

  if [[ "$has_demo" -eq 1 ]] && demo_launches_manager; then
    has_manager=1
  fi

  if [[ ("$has_teleop" -eq 1 || "$has_foxglove" -eq 1) && "$has_manager" -eq 0 ]]; then
    echo "${YLW}Warning:${RST} teleop/foxglove selected without manager; proceeding anyway."
  fi
}

apply_selection_flags() {
  WITH_WEBOTS=0
  WITH_MANAGER=0
  WITH_TELEOP=0
  WITH_FOXGLOVE=0
  WITH_TOOLS=0
  WITH_RQT=0
  WITH_YOLO_VISION=0
  WITH_LOCALIZATION=0
  WITH_BALL_HEAD_TRACKING=0
  WITH_BALL_LOCALIZER=0
  WITH_ACTION_EDITOR=0
  WITH_ACTION_WEB=0
  WITH_VISION_LAB=0
  WITH_DEMO=0
  WITH_OFFSET_TUNER=0
  MANAGER_MODE=""
  MANAGER_CONFLICT=0

  local set_manager_mode
  set_manager_mode() {
    local mode="$1"
    if [[ -n "$MANAGER_MODE" && "$MANAGER_MODE" != "$mode" ]]; then
      MANAGER_CONFLICT=1
      return
    fi
    MANAGER_MODE="$mode"
  }

  local item
  for item in "$@"; do
    case "$item" in
      webots) WITH_WEBOTS=1 ;;
      manager) WITH_MANAGER=1 ;;
      manager_sim) WITH_MANAGER=1; set_manager_mode "sim" ;;
      manager_real) WITH_MANAGER=1; set_manager_mode "real" ;;
      offset_tuner) WITH_OFFSET_TUNER=1 ;;
      demo) WITH_DEMO=1 ;;
      teleop) WITH_TELEOP=1 ;;
      foxglove) WITH_FOXGLOVE=1 ;;
      tools) WITH_TOOLS=1 ;;
      rqt_image_view) WITH_RQT=1 ;;
      yolo_vision) WITH_YOLO_VISION=1 ;;
      localization) WITH_LOCALIZATION=1 ;;
      ball_head_tracking) WITH_BALL_HEAD_TRACKING=1 ;;
      ball_localizer) WITH_BALL_LOCALIZER=1 ;;
      action_editor) WITH_ACTION_EDITOR=1 ;;
      action_web) WITH_ACTION_WEB=1 ;;
      vision_lab) WITH_VISION_LAB=1 ;;
    esac
  done
}

resolve_selection() {
  local use_explicit="$1"
  local use_usual="$2"
  local save_usual="$3"

  local -a selected=()
  local from_interactive=0

  if [[ "$use_explicit" -eq 1 ]]; then
    if [[ "$WITH_OFFSET_TUNER" -eq 1 ]]; then
      selected+=(offset_tuner)
    else
      local needs_base_stack=0
      if [[ "$WITH_TELEOP" -eq 1 || "$WITH_FOXGLOVE" -eq 1 || "$WITH_TOOLS" -eq 1 || "$WITH_YOLO_VISION" -eq 1 || "$WITH_LOCALIZATION" -eq 1 || "$WITH_BALL_HEAD_TRACKING" -eq 1 || "$WITH_BALL_LOCALIZER" -eq 1 || "$WITH_ACTION_EDITOR" -eq 1 || "$WITH_ACTION_WEB" -eq 1 ]]; then
        needs_base_stack=1
      fi

      if [[ "$WITH_DEMO" -eq 1 ]]; then
        if [[ "$needs_base_stack" -eq 1 ]]; then
          selected+=(webots)
        fi
      elif [[ "$WITH_VISION_LAB" -eq 0 || "$needs_base_stack" -eq 1 ]]; then
        selected+=(webots manager)
      fi
      [[ "$WITH_DEMO" -eq 1 ]] && selected+=(demo)
      [[ "$WITH_TELEOP" -eq 1 ]] && selected+=(teleop)
      [[ "$WITH_FOXGLOVE" -eq 1 ]] && selected+=(foxglove)
      [[ "$WITH_TOOLS" -eq 1 ]] && selected+=(tools)
      [[ "$WITH_YOLO_VISION" -eq 1 ]] && selected+=(yolo_vision)
      [[ "$WITH_LOCALIZATION" -eq 1 ]] && selected+=(localization)
      [[ "$WITH_BALL_HEAD_TRACKING" -eq 1 ]] && selected+=(ball_head_tracking)
      [[ "$WITH_BALL_LOCALIZER" -eq 1 ]] && selected+=(ball_localizer)
      [[ "$WITH_ACTION_EDITOR" -eq 1 ]] && selected+=(action_editor)
      [[ "$WITH_ACTION_WEB" -eq 1 ]] && selected+=(action_web)
      [[ "$WITH_VISION_LAB" -eq 1 ]] && selected+=(vision_lab)
    fi
  elif [[ "$use_usual" -eq 1 ]]; then
    mapfile -t selected < <(load_usual_selection)
  else
    from_interactive=1
    mapfile -t selected < <(interactive_selection)
  fi

  if [[ "$from_interactive" -eq 1 ]]; then
    if [[ "${#selected[@]}" -eq 0 ]]; then
      MENU_ACTION="cancel"
      return 0
    fi
    if [[ "${#selected[@]}" -eq 1 && "${selected[0]}" == "$SELECTION_CANCELLED" ]]; then
      MENU_ACTION="cancel"
      return 0
    fi
  fi

  if [[ "${#selected[@]}" -eq 0 ]]; then
    if [[ -n "$MENU_ACTION" ]]; then
      return 0
    fi
    mapfile -t selected < <(load_usual_selection)
    if [[ "${#selected[@]}" -eq 0 ]]; then
      selected=(manager_sim)
    fi
  fi

  local has_demo=0
  for item in "${selected[@]}"; do
    case "$item" in
      demo) has_demo=1 ;;
    esac
  done
  if [[ "$has_demo" -eq 1 ]] && demo_launches_manager; then
    local -a filtered=()
    for item in "${selected[@]}"; do
      case "$item" in
        manager|manager_sim|manager_real) ;;
        *) filtered+=("$item") ;;
      esac
    done
    selected=("${filtered[@]}")
  fi

  validate_selection "${selected[@]}"

  # Always remember the last selection (used to preselect next time).
  if [[ "${OP3_SAVE_LAST_SELECTION:-1}" == "1" && "${DRY_RUN:-0}" -ne 1 ]]; then
    save_last_selection "${selected[@]}"
  fi

  if [[ "$save_usual" -eq 1 ]]; then
    save_usual_selection "${selected[@]}"
  fi

  apply_selection_flags "${selected[@]}"

  if [[ "$MANAGER_CONFLICT" -eq 1 ]]; then
    echo "${YLW}Warning:${RST} Select only one manager type (sim or real)."
    MENU_ACTION="cancel"
    return 0
  fi

  if [[ "$WITH_OFFSET_TUNER" -eq 1 && "$WITH_MANAGER" -eq 1 ]]; then
    echo "${YLW}Warning:${RST} Offset Tuner cannot run alongside manager. Stop manager and select only Offset Tuner."
    MENU_ACTION="cancel"
    return 0
  fi

  if [[ "$MANAGER_MODE" == "sim" ]]; then
    PROFILE="webots"
    apply_profile "$PROFILE"
  elif [[ "$MANAGER_MODE" == "real" ]]; then
    PROFILE="real_robot"
    apply_profile "$PROFILE"
  fi

  if [[ "$WITH_OFFSET_TUNER" -eq 1 ]]; then
    PROFILE="real_robot"
    apply_profile "$PROFILE"
  fi
}
