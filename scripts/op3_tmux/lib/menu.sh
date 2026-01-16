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
  echo "${BOLD}Choose what to run:${RST}"
  echo "  1) Base: Webots + Manager"
  echo "  2) Base + Teleop"
  echo "  3) Base + Foxglove"
  echo "  4) Base + Teleop + Foxglove"
  echo "  5) Base + Tools"
  echo "  6) Base + RQT Image View"
  echo "  7) Base + Teleop + Foxglove + Tools + RQT"
  echo "  8) Base + Vision + Ball Localizer"
  echo "  9) Base + Action Editor"
  echo " 10) Base + Action Web"
  echo " 11) Attach to existing session"
  echo " 12) Exit (kill session)"
  echo " 13) Status"
  echo "  0) Quit"
  echo
  read -r -p "Select [0-13]: " choice

  case "${choice:-0}" in
    1) printf "%s\n" webots manager ;;
    2) printf "%s\n" webots manager teleop ;;
    3) printf "%s\n" webots manager foxglove ;;
    4) printf "%s\n" webots manager teleop foxglove ;;
    5) printf "%s\n" webots manager tools ;;
    6) printf "%s\n" webots manager rqt_image_view ;;
    7) printf "%s\n" webots manager teleop foxglove tools rqt_image_view ;;
    8) printf "%s\n" webots manager yolo_vision ball_localizer ;;
    9) printf "%s\n" webots manager action_editor ;;
    10) printf "%s\n" webots manager action_web ;;
    11) MENU_ACTION="attach" ;;
    12) MENU_ACTION="exit" ;;
    13) MENU_ACTION="status" ;;
    0) MENU_ACTION="quit" ;;
    *) echo "${YLW}Invalid choice.${RST}"; exit 1 ;;
  esac
}
