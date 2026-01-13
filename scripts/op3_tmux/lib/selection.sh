COMPONENTS=(webots manager teleop foxglove tools rqt_image_view)
COMPONENTS=(webots manager teleop foxglove tools rqt_image_view)
MENU_ACTION=""

usual_file_path() {
  echo "${HOME}/.config/op3-stack/usual.txt"
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

save_usual_selection() {
  local file
  file="$(usual_file_path)"
  mkdir -p "$(dirname "$file")"
  printf "%s\n" "$@" > "$file"
}

join_by_comma() {
  local IFS=,
  echo "$*"
}

select_with_gum() {
  local selected_csv=""
  if [[ -f "$(usual_file_path)" ]]; then
    mapfile -t _usual < <(load_usual_selection)
    if [[ ${#_usual[@]} -gt 0 ]]; then
      selected_csv="$(join_by_comma "${_usual[@]}")"
    fi
  else
    selected_csv="webots,manager"
  fi



  if [[ -n "$selected_csv" ]]; then
    gum choose --no-limit --selected "$selected_csv" --header "Use <space> to toggle, <enter> to start" "${COMPONENTS[@]}" || true
  else
    gum choose --no-limit --header "Use <space>  to toggle, <enter> to start" "${COMPONENTS[@]}" || true
  fi
}

interactive_selection() {
  banner >&2
  if command -v gum >/dev/null 2>&1; then
    select_with_gum
    return 0
  fi

  echo "${YLW}gum is not installed.${RST} Install: sudo apt install gum"
  fallback_menu_selection
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

  local item
  for item in "$@"; do
    case "$item" in
      webots) WITH_WEBOTS=1 ;;
      manager) WITH_MANAGER=1 ;;
      teleop) WITH_TELEOP=1 ;;
      foxglove) WITH_FOXGLOVE=1 ;;
      tools) WITH_TOOLS=1 ;;
      rqt_image_view) WITH_RQT=1 ;;
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
    [[ "$WITH_TELEOP" -eq 1 ]] && selected+=(teleop)
    [[ "$WITH_FOXGLOVE" -eq 1 ]] && selected+=(foxglove)
    [[ "$WITH_TOOLS" -eq 1 ]] && selected+=(tools)
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

  validate_selection "${selected[@]}"

  if [[ "$save_usual" -eq 1 ]]; then
    save_usual_selection "${selected[@]}"
  fi

  apply_selection_flags "${selected[@]}"
}
