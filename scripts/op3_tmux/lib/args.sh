help() {
  banner
  cat <<EOF
${BOLD}Usage${RST}
  ${BOLD}./$(basename "$0")${RST}                 Interactive menu
  ${BOLD}./$(basename "$0") --base${RST}          Start: webots + manager
  ${BOLD}./$(basename "$0") --teleop${RST}        Start: webots + manager + teleop
  ${BOLD}./$(basename "$0") --foxglove${RST}      Start: webots + manager + foxglove
  ${BOLD}./$(basename "$0") --all${RST}           Start: webots + manager + teleop + foxglove
  ${BOLD}./$(basename "$0") --vision${RST}        Start: webots + manager + yolo_vision
  ${BOLD}./$(basename "$0") --ball${RST}          Start: webots + manager + yolo_vision + ball_localizer
  ${BOLD}./$(basename "$0") --ball-localizer${RST}Start: webots + manager + ball_localizer
  ${BOLD}./$(basename "$0") --action-editor${RST} Start: webots + manager + action_editor
  ${BOLD}./$(basename "$0") --demo${RST}          Start: webots + manager + demo
  ${BOLD}./$(basename "$0") --demo-soccer${RST}   Start: webots + manager + demo (soccer)
  ${BOLD}./$(basename "$0") --demo-vision${RST}   Start: webots + manager + demo (vision)
  ${BOLD}./$(basename "$0") --demo-action${RST}   Start: webots + manager + demo (action)
  ${BOLD}./$(basename "$0") --studio${RST}        Start: webots + manager + bascorro studio
  ${BOLD}./$(basename "$0") --action-web${RST}    Alias for --studio
  ${BOLD}./$(basename "$0") --localization${RST}  Start: webots + manager + localization (+ rviz)
  ${BOLD}./$(basename "$0") --tools${RST}         Start: add quick ROS tools pane
  ${BOLD}./$(basename "$0") -u${RST}              Start: usual selection from ~/.config/op3-stack/usual.txt
  ${BOLD}./$(basename "$0") --save-usual${RST}    Save current selection as usual
  ${BOLD}./$(basename "$0") --stop${RST}          Publish stop + zero then kill session
  ${BOLD}./$(basename "$0") --restart <comp>${RST}Restart one component pane
  ${BOLD}./$(basename "$0") --attach${RST}        Attach to running session
  ${BOLD}./$(basename "$0") --status${RST}        Session status
  ${BOLD}./$(basename "$0") --doctor${RST}        Health check (deps + workspace)
  ${BOLD}./$(basename "$0") --install-deps${RST}  Install apt + rosdep dependencies
  ${BOLD}./$(basename "$0") --build${RST}         Build workspace (colcon)
  ${BOLD}./$(basename "$0") --docker-build${RST}  Build Docker image
  ${BOLD}./$(basename "$0") --docker-run${RST}    Run container (docker run)
  ${BOLD}./$(basename "$0") --docker-up${RST}     Run docker-compose up
  ${BOLD}./$(basename "$0") --docker-down${RST}   Stop docker-compose
  ${BOLD}./$(basename "$0") --exit${RST}          Stop everything (kill session)
  ${BOLD}./$(basename "$0") -x${RST}              Stop everything (alias for --exit)

${BOLD}Options${RST}
  --ws PATH            Workspace root (default: ${WS})
  --setup PATH         Setup script (auto-detect if omitted)
  --session NAME       tmux session name (default: ${SESSION})
  --delay SEC          Delay between starting Webots then Manager (default: ${START_DELAY_SEC})
  --profile NAME       Profile: webots | real_robot (default: ${PROFILE})
  --tools              Enable quick ROS tools pane
  --vision             Enable vision stack (yolo_vision)
  --ball               Enable vision + ball_localizer
  --ball-localizer     Enable ball_localizer
  --action-editor      Enable action editor (bridge + editor)
  --demo               Enable demo stack (op3_demo)
  --demo-soccer        Enable demo stack and switch to soccer mode
  --demo-vision        Enable demo stack and switch to vision mode
  --demo-action        Enable demo stack and switch to action mode
  --studio             Enable bascorro studio UI (bridge + apply node + Vite)
  --action-web         Alias for --studio
  --localization       Enable localization stack (soccer_localization + rviz)
  --restart <comp>      comp: webots|manager|teleop|foxglove|tools|rqt_image_view|yolo_vision|localization|ball_localizer|action_editor|action_web|demo
  --doctor              Health check (deps + workspace)
  --install-deps        Install apt + rosdep dependencies
  --build               Build workspace (colcon)
  --docker-build        Build Docker image
  --docker-run          Run container (docker run)
  --docker-up           Run docker-compose up
  --docker-down         Stop docker-compose
  --usual, -u          Use saved selection from ~/.config/op3-stack/usual.txt
  --save-usual         Save selected components to ~/.config/op3-stack/usual.txt
  --dry-run            Print what would run (no changes)

${BOLD}Defaults (env override)${RST}
  WEBOTS:  ${WEBOTS_CMD}
  MANAGER: ${MANAGER_CMD}
  ACTION_EDITOR: ${ACTION_EDITOR_CMD}
  STUDIO: ${ACTION_WEB_CMD}
  DEMO: ${DEMO_CMD}
  ACTION_FILE: ${ACTION_FILE_PATH}
  ACTION_EDITOR_LOG: ${ACTION_EDITOR_LOG}

${BOLD}Env overrides (recommended)${RST}
  OP3_WS, OP3_SETUP, OP3_SESSION, OP3_START_DELAY_SEC
  OP3_WEBOTS_CMD, OP3_MANAGER_CMD, OP3_TELEOP_CMD, OP3_FOXGLOVE_CMD
  OP3_YOLO_VISION_CMD, OP3_LOCALIZATION_CMD, OP3_BALL_LOCALIZER_CMD
  OP3_ACTION_EDITOR_CMD, OP3_ACTION_FILE, OP3_ACTION_FILE_SEED, OP3_ACTION_EDITOR_LOG, OP3_DEMO_CMD
  OP3_DEMO_MODE, OP3_DEMO_WAIT_STEP_SEC, OP3_DEMO_WAIT_STEPS
  OP3_STUDIO_CMD, OP3_ACTION_WEB_CMD
  OP3_SHELL_RUNNER (e.g. "zsh -lc" or "bash -lc")
  OP3_PREFER_ZSH (set 1 to prefer zsh + setup.zsh)
  OP3_PREFS_FILE (default: ~/.config/op3-stack/prefs.sh)
  WEBOTS_HOME, OP3_PROFILE, OP3_TOOLS_CMD, OP3_RQT_CMD
  OP3_DOCKER_TAG, OP3_DOCKER_WITH_WEBOTS, OP3_DOCKER_WEBOTS_VERSION, OP3_DOCKER_WEBOTS_PACKAGE_PREFIX
  OP3_DOCKER_BUILD_FLAGS, OP3_DOCKER_RUN_FLAGS, OP3_DOCKER_UP_FLAGS
  OP3_DOCKER_MOUNT_MODE (none|src|cache|full), OP3_DOCKER_SRC_RO (1|0)
  OP3_SAVE_LAST_SELECTION (default: 1; writes ~/.config/op3-stack/last.txt)

${BOLD}Profiles${RST}
  webots     Webots simulation defaults (current commands, delay, domain)
  real_robot Lower startup delay and different default ROS_DOMAIN_ID

${BOLD}Notes${RST}
- Pane order: ${BOLD}Pane 0 = Webots${RST}, ${BOLD}Pane 1 = Manager${RST}.
- Panes are created only for selected components; pane count matches selection count.
- Interactive selection uses ${BOLD}gum${RST} if available; install with: sudo apt install gum
- Kill everything: ${BOLD}./$(basename "$0") --exit${RST}
EOF
}

op3_tmux_dispatch() {
  # -------------------- Args --------------------
  WITH_TELEOP=0
  WITH_FOXGLOVE=0
  WITH_TOOLS=0
  WITH_WEBOTS=0
  WITH_MANAGER=0
  WITH_RQT=0
  WITH_YOLO_VISION=0
  WITH_LOCALIZATION=0
  WITH_BALL_LOCALIZER=0
  WITH_ACTION_EDITOR=0
  WITH_ACTION_WEB=0
  WITH_DEMO=0
  DEMO_MODE=""
  DO_ATTACH=0
  DO_STATUS=0
  DO_DOCTOR=0
  DO_INSTALL_DEPS=0
  DO_BUILD=0
  DO_DOCKER_BUILD=0
  DO_DOCKER_RUN=0
  DO_DOCKER_UP=0
  DO_DOCKER_DOWN=0
  DO_EXIT=0
  DO_STOP=0
  DO_USUAL=0
  DO_SAVE_USUAL=0
  RESTART_COMPONENT=""
  DRY_RUN=0
  START_EXPLICIT=0

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --ws) WS="$2"; shift 2 ;;
      --setup) SETUP="$2"; shift 2 ;;
      --session) SESSION="$2"; shift 2 ;;
      --delay) START_DELAY_SEC="$2"; shift 2 ;;
      --profile) PROFILE="$2"; shift 2 ;;
      --base) START_EXPLICIT=1; shift ;;
      --teleop) WITH_TELEOP=1; START_EXPLICIT=1; shift ;;
      --foxglove) WITH_FOXGLOVE=1; START_EXPLICIT=1; shift ;;
      --all) WITH_TELEOP=1; WITH_FOXGLOVE=1; START_EXPLICIT=1; shift ;;
      --vision) WITH_YOLO_VISION=1; START_EXPLICIT=1; shift ;;
      --ball) WITH_YOLO_VISION=1; WITH_BALL_LOCALIZER=1; START_EXPLICIT=1; shift ;;
      --ball-localizer) WITH_BALL_LOCALIZER=1; START_EXPLICIT=1; shift ;;
      --action-editor) WITH_ACTION_EDITOR=1; START_EXPLICIT=1; shift ;;
      --demo) WITH_DEMO=1; START_EXPLICIT=1; shift ;;
      --demo-soccer) WITH_DEMO=1; DEMO_MODE="soccer"; START_EXPLICIT=1; shift ;;
      --demo-vision) WITH_DEMO=1; DEMO_MODE="vision"; START_EXPLICIT=1; shift ;;
      --demo-action) WITH_DEMO=1; DEMO_MODE="action"; START_EXPLICIT=1; shift ;;
      --studio|--action-web) WITH_ACTION_WEB=1; START_EXPLICIT=1; shift ;;
      --localization) WITH_LOCALIZATION=1; START_EXPLICIT=1; shift ;;
      --tools) WITH_TOOLS=1; START_EXPLICIT=1; shift ;;
    --usual|-u) DO_USUAL=1; shift ;;
    --save-usual) DO_SAVE_USUAL=1; shift ;;
      --attach) DO_ATTACH=1; shift ;;
      --status) DO_STATUS=1; shift ;;
      --doctor) DO_DOCTOR=1; shift ;;
      --install-deps) DO_INSTALL_DEPS=1; shift ;;
      --build) DO_BUILD=1; shift ;;
      --docker-build) DO_DOCKER_BUILD=1; shift ;;
      --docker-run) DO_DOCKER_RUN=1; shift ;;
      --docker-up) DO_DOCKER_UP=1; shift ;;
      --docker-down) DO_DOCKER_DOWN=1; shift ;;
    --exit|-x) DO_EXIT=1; shift ;;
      --stop) DO_STOP=1; shift ;;
      --restart) RESTART_COMPONENT="$2"; shift 2 ;;
      --dry-run) DRY_RUN=1; shift ;;
      -h|--help) apply_profile "$PROFILE"; help; exit 0 ;;
      *) die "Unknown argument: $1 (use --help)" ;;
    esac
  done

  apply_profile "$PROFILE"

  if [[ "$DO_EXIT" -eq 1 ]]; then action_exit; exit 0; fi
  if [[ "$DO_STATUS" -eq 1 ]]; then action_status; exit 0; fi
  if [[ "$DO_DOCTOR" -eq 1 ]]; then action_doctor; exit $?; fi
  if [[ "$DO_INSTALL_DEPS" -eq 1 ]]; then action_install_deps; exit $?; fi
  if [[ "$DO_BUILD" -eq 1 ]]; then action_build; exit $?; fi
  if [[ "$DO_DOCKER_BUILD" -eq 1 ]]; then action_docker_build; exit $?; fi
  if [[ "$DO_DOCKER_RUN" -eq 1 ]]; then action_docker_run; exit $?; fi
  if [[ "$DO_DOCKER_UP" -eq 1 ]]; then action_docker_up; exit $?; fi
  if [[ "$DO_DOCKER_DOWN" -eq 1 ]]; then action_docker_down; exit $?; fi
  if [[ "$DO_ATTACH" -eq 1 ]]; then action_attach; exit 0; fi
  if [[ "$DO_STOP" -eq 1 ]]; then action_stop; exit 0; fi
  if [[ -n "$RESTART_COMPONENT" ]]; then restart_component "$RESTART_COMPONENT"; exit 0; fi

  resolve_selection "$START_EXPLICIT" "$DO_USUAL" "$DO_SAVE_USUAL"

  if [[ -n "$MENU_ACTION" ]]; then
    case "$MENU_ACTION" in
      attach) action_attach ;;
      status) action_status ;;
      exit) action_exit ;;
      cancel) exit 2 ;;
      quit) exit 0 ;;
      *) die "Unknown menu action: $MENU_ACTION" ;;
    esac
    exit 0
  fi

  start_stack "$WITH_WEBOTS" "$WITH_MANAGER" "$WITH_TELEOP" "$WITH_FOXGLOVE" "$WITH_TOOLS" "$WITH_RQT" "$WITH_YOLO_VISION" "$WITH_LOCALIZATION" "$WITH_BALL_LOCALIZER" "$WITH_ACTION_EDITOR" "$WITH_ACTION_WEB" "$WITH_DEMO" "$DRY_RUN"
}
