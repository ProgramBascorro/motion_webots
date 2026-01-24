COMPONENTS=(webots manager demo teleop action_web rqt_image_view yolo_vision localization ball_localizer action_editor foxglove tools)
MENU_ACTION=""

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
    printf "%s\n" webots manager
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
  if [[ -f "$(last_file_path)" ]]; then
    mapfile -t _last < <(load_last_selection)
    if [[ ${#_last[@]} -gt 0 ]]; then
      selected_csv="$(join_by_comma "${_last[@]}")"
    fi
  elif [[ -f "$(usual_file_path)" ]]; then
    mapfile -t _usual < <(load_usual_selection)
    if [[ ${#_usual[@]} -gt 0 ]]; then
      selected_csv="$(join_by_comma "${_usual[@]}")"
    fi
  fi
  [[ -z "$selected_csv" ]] && selected_csv="webots,manager"

  if [[ -n "$selected_csv" ]]; then
    gum choose --no-limit --selected "$selected_csv" --header "Use <space> to toggle, <enter> to start" "${COMPONENTS[@]}"
  else
    gum choose --no-limit --header "Use <space>  to toggle, <enter> to start" "${COMPONENTS[@]}"
  fi
  if [[ "$?" -ne 0 ]]; then
    MENU_ACTION="cancel"
    return 1
  fi
  return 0
}

interactive_selection() {
  # banner >&2
  if ! command -v gum >/dev/null 2>&1; then
    echo "${RED}ERROR:${RST} gum is not installed." >&2
    echo "${DIM}Install:${RST} sudo apt install gum" >&2
    exit 1
  fi
  select_with_gum
}

validate_selection() {
  local has_manager=0
  local has_teleop=0
  local has_foxglove=0

  local item
  for item in "$@"; do
    case "$item" in
      manager) has_manager=1 ;;
      teleop) has_teleop=1 ;;
      foxglove) has_foxglove=1 ;;
    esac
  done

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
  WITH_BALL_LOCALIZER=0
  WITH_ACTION_EDITOR=0
  WITH_ACTION_WEB=0
  WITH_DEMO=0

  local item
  for item in "$@"; do
    case "$item" in
      webots) WITH_WEBOTS=1 ;;
      manager) WITH_MANAGER=1 ;;
      demo) WITH_DEMO=1 ;;
      teleop) WITH_TELEOP=1 ;;
      foxglove) WITH_FOXGLOVE=1 ;;
      tools) WITH_TOOLS=1 ;;
      rqt_image_view) WITH_RQT=1 ;;
      yolo_vision) WITH_YOLO_VISION=1 ;;
      localization) WITH_LOCALIZATION=1 ;;
      ball_localizer) WITH_BALL_LOCALIZER=1 ;;
      action_editor) WITH_ACTION_EDITOR=1 ;;
      action_web) WITH_ACTION_WEB=1 ;;
    esac
  done
}

resolve_selection() {
  local use_explicit="$1"
  local use_usual="$2"
  local save_usual="$3"

  local -a selected=()

  if [[ "$use_explicit" -eq 1 ]]; then
    selected+=(webots manager)
    [[ "$WITH_DEMO" -eq 1 ]] && selected+=(demo)
    [[ "$WITH_TELEOP" -eq 1 ]] && selected+=(teleop)
    [[ "$WITH_FOXGLOVE" -eq 1 ]] && selected+=(foxglove)
    [[ "$WITH_TOOLS" -eq 1 ]] && selected+=(tools)
    [[ "$WITH_YOLO_VISION" -eq 1 ]] && selected+=(yolo_vision)
    [[ "$WITH_LOCALIZATION" -eq 1 ]] && selected+=(localization)
    [[ "$WITH_BALL_LOCALIZER" -eq 1 ]] && selected+=(ball_localizer)
    [[ "$WITH_ACTION_EDITOR" -eq 1 ]] && selected+=(action_editor)
    [[ "$WITH_ACTION_WEB" -eq 1 ]] && selected+=(action_web)
  elif [[ "$use_usual" -eq 1 ]]; then
    mapfile -t selected < <(load_usual_selection)
  else
    mapfile -t selected < <(interactive_selection)
  fi

  if [[ "${#selected[@]}" -eq 0 ]]; then
    if [[ -n "$MENU_ACTION" ]]; then
      return 0
    fi
    mapfile -t selected < <(load_usual_selection)
    if [[ "${#selected[@]}" -eq 0 ]]; then
      selected=(webots manager)
    fi
  fi

  local has_demo=0
  local has_manager=0
  for item in "${selected[@]}"; do
    case "$item" in
      demo) has_demo=1 ;;
      manager) has_manager=1 ;;
    esac
  done
  if [[ "$has_demo" -eq 1 && "$has_manager" -eq 0 ]]; then
    selected+=(manager)
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
}
