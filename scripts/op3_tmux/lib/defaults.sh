# -------------------- Defaults --------------------
SESSION="${OP3_SESSION:-op3}"
WS_DEFAULT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
WS="${OP3_WS:-$WS_DEFAULT}"
SETUP="${OP3_SETUP:-}"                        # if empty, auto-detect in WS
OP3_ENV_SCRIPT="${OP3_ENV_SCRIPT:-$WS/scripts/op3_env.sh}"

PROFILE="${OP3_PROFILE:-webots}"

# Best-effort detection of the user's login shell (useful when this launcher is executed by bash).
if [[ -z "${OP3_PARENT_SHELL:-}" && -n "${SHELL:-}" ]]; then
  OP3_PARENT_SHELL="${SHELL##*/}"
fi

# Load persisted preferences (shell runner, etc).
OP3_CONFIG_DIR="${OP3_CONFIG_DIR:-${HOME:-}/.config/op3-stack}"
OP3_PREFS_FILE="${OP3_PREFS_FILE:-$OP3_CONFIG_DIR/prefs.sh}"
if [[ -n "${HOME:-}" && -f "$OP3_PREFS_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$OP3_PREFS_FILE"
fi
SETUP="${OP3_SETUP:-$SETUP}"

# Your requested defaults:
WEBOTS_CMD="${OP3_WEBOTS_CMD:-ros2 launch op3_webots_ros2 robot_launch.py}"
MANAGER_CMD="${OP3_MANAGER_CMD:-ros2 launch op3_manager op3_simulation.launch.py}"

TELEOP_CMD="${OP3_TELEOP_CMD:-ros2 launch op3_joy_teleop op3_joy_teleop.launch.py}"
FOXGLOVE_CMD="${OP3_FOXGLOVE_CMD:-ros2 launch foxglove_bridge foxglove_bridge_launch.xml}"
TOOLS_CMD="${OP3_TOOLS_CMD:-echo '[tools] /joy hz'; (ros2 topic hz /joy &) ; echo '[tools] /joint_states hz'; (ros2 topic hz /joint_states &) ; echo '[tools] /robotis/walking/command echo'; ros2 topic echo /robotis/walking/command}"
RQT_CMD="${OP3_RQT_CMD:-ros2 run rqt_image_view rqt_image_view}"

# Vision/localization stack
YOLO_VISION_CMD="${OP3_YOLO_VISION_CMD:-ros2 launch op3_yolo_vision yolo.launch.py}"
LOCALIZATION_CMD="${OP3_LOCALIZATION_CMD:-ros2 launch soccer_localization localization.launch.py use_rviz:=false}"
BALL_HEAD_TRACKING_CMD="${OP3_BALL_HEAD_TRACKING_CMD:-ros2 launch op3_ball_localization yolo_scan_only.launch.py}"
BALL_LOCALIZER_CMD="${OP3_BALL_LOCALIZER_CMD:-ros2 launch op3_ball_localization ball_localizer.launch.py}"

# Offset tuner
OFFSET_TUNER_CMD="${OP3_OFFSET_TUNER_CMD:-ros2 launch op3_offset_tuner_server op3_offset_tuner_server.launch.xml}"

# Action editor
ACTION_FILE_SEED="${OP3_ACTION_FILE_SEED:-$WS/src/ROBOTIS-OP3/op3_action_module/data/motion_4095.bin}"
ACTION_FILE_PATH="${OP3_ACTION_FILE:-$WS/src/ROBOTIS-OP3/op3_action_module/data/motion_custom.bin}"
ACTION_EDITOR_LOG="${OP3_ACTION_EDITOR_LOG:-/tmp/op3_action_editor_bridge.log}"
ACTION_EDITOR_CMD="${OP3_ACTION_EDITOR_CMD:-bridge_log='${ACTION_EDITOR_LOG}'; bridge_pid=0; ros2 run op3_action_editor bridge_webots.py >\"\$bridge_log\" 2>&1 & bridge_pid=\$!; ros2 run op3_action_editor webots_executor.py; if [ \$bridge_pid -ne 0 ]; then kill \$bridge_pid; wait \$bridge_pid 2>/dev/null; fi}"

# Action web
ACTION_WEB_CMD="${OP3_ACTION_WEB_CMD:-${OP3_STUDIO_CMD:-$WS/scripts/bascorro_studio.sh}}"
VISION_LAB_CMD="${OP3_VISION_LAB_CMD:-$WS/scripts/vision_lab.sh}"

# Demo
DEMO_CMD="${OP3_DEMO_CMD:-ros2 launch op3_demo demo_yolo.launch.xml}"
DEMO_MODE="${OP3_DEMO_MODE:-}"
DEMO_WAIT_STEP_SEC="${OP3_DEMO_WAIT_STEP_SEC:-0.5}"
DEMO_WAIT_STEPS="${OP3_DEMO_WAIT_STEPS:-40}"

ROS_DOMAIN_ID_DEFAULT_ENV="${ROS_DOMAIN_ID_DEFAULT:-}"
ROS_DOMAIN_ID_DEFAULT="${ROS_DOMAIN_ID_DEFAULT_ENV:-0}"
START_DELAY_SEC="${OP3_START_DELAY_SEC:-4.0}"

# Auto pick shell runner based on setup file unless user overrides OP3_SHELL_RUNNER
SHELL_RUNNER="${OP3_SHELL_RUNNER:-}"
