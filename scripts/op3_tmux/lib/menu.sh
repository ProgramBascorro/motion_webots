fallback_menu_selection() {
  MENU_ACTION=""

  auto_detect_setup
  auto_detect_shell_runner

  banner
  echo "${BOLD}Workspace:${RST} $WS"
  echo "${BOLD}Setup:${RST}     $SETUP"
  echo "${BOLD}Shell:${RST}     $SHELL_RUNNER"
  echo "${BOLD}Session:${RST}   $SESSION"
  echo "${BOLD}Delay:${RST}     ${START_DELAY_SEC}s (webots → manager)"
  echo "${BOLD}Profile:${RST}   $PROFILE"
  echo
  if ! command -v gum >/dev/null 2>&1; then
    echo "${RED}ERROR:${RST} gum is not installed." >&2
    echo "${DIM}Install:${RST} sudo apt install gum" >&2
    exit 1
  fi

  local choice
  choice="$(gum choose --header "Choose what to run" \
    "Base: Webots + Manager" \
    "Base + Teleop" \
    "Base + Foxglove" \
    "Base + Teleop + Foxglove" \
    "Base + Tools" \
    "Base + RQT Image View" \
    "Base + Teleop + Foxglove + Tools + RQT" \
    "Base + Vision + Ball Localizer" \
    "Base + Action Editor" \
    "Base + Action Web" \
    "Attach to existing session" \
    "Exit (kill session)" \
    "Status" \
    "Quit")"

  case "$choice" in
    "Base: Webots + Manager") printf "%s\n" webots manager ;;
    "Base + Teleop") printf "%s\n" webots manager teleop ;;
    "Base + Foxglove") printf "%s\n" webots manager foxglove ;;
    "Base + Teleop + Foxglove") printf "%s\n" webots manager teleop foxglove ;;
    "Base + Tools") printf "%s\n" webots manager tools ;;
    "Base + RQT Image View") printf "%s\n" webots manager rqt_image_view ;;
    "Base + Teleop + Foxglove + Tools + RQT") printf "%s\n" webots manager teleop foxglove tools rqt_image_view ;;
    "Base + Vision + Ball Localizer") printf "%s\n" webots manager yolo_vision ball_localizer ;;
    "Base + Action Editor") printf "%s\n" webots manager action_editor ;;
    "Base + Action Web") printf "%s\n" webots manager action_web ;;
    "Attach to existing session") MENU_ACTION="attach" ;;
    "Exit (kill session)") MENU_ACTION="exit" ;;
    "Status") MENU_ACTION="status" ;;
    "Quit"|*) MENU_ACTION="quit" ;;
  esac
}
