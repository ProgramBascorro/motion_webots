# -------------------- Defaults --------------------
SESSION="${OP3_SESSION:-op3}"
WS="${OP3_WS:-/home/farhan/Projects/Surgical_lokalisasi_bismillah/motion_webots_farhan_coba}"
SETUP="${OP3_SETUP:-}"                        # if empty, auto-detect in WS
OP3_ENV_SCRIPT="${OP3_ENV_SCRIPT:-$WS/scripts/op3_env.sh}"

PROFILE="${OP3_PROFILE:-webots}"

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
BALL_LOCALIZER_CMD="${OP3_BALL_LOCALIZER_CMD:-ros2 launch op3_ball_localization ball_localizer.launch.py}"

# Action editor
ACTION_FILE_SEED="${OP3_ACTION_FILE_SEED:-$WS/src/ROBOTIS-OP3/op3_action_module/data/motion_4095.bin}"
ACTION_FILE_PATH="${OP3_ACTION_FILE:-$WS/src/ROBOTIS-OP3/op3_action_module/data/motion_custom.bin}"
ACTION_EDITOR_LOG="${OP3_ACTION_EDITOR_LOG:-/tmp/op3_action_editor_bridge.log}"
ACTION_EDITOR_CMD="${OP3_ACTION_EDITOR_CMD:-bridge_log='${ACTION_EDITOR_LOG}'; bridge_pid=0; ros2 run op3_action_editor bridge_webots.py >\"\$bridge_log\" 2>&1 & bridge_pid=\$!; ros2 run op3_action_editor webots_executor.py; if [ \$bridge_pid -ne 0 ]; then kill \$bridge_pid; wait \$bridge_pid 2>/dev/null; fi}"

# Action web
ACTION_WEB_CMD="${OP3_ACTION_WEB_CMD:-$WS/scripts/op3_action_web.sh}"

ROS_DOMAIN_ID_DEFAULT_ENV="${ROS_DOMAIN_ID_DEFAULT:-}"
ROS_DOMAIN_ID_DEFAULT="${ROS_DOMAIN_ID_DEFAULT_ENV:-0}"
START_DELAY_SEC="${OP3_START_DELAY_SEC:-4.0}"

# Auto pick shell runner based on setup file unless user overrides OP3_SHELL_RUNNER
SHELL_RUNNER="${OP3_SHELL_RUNNER:-}"
