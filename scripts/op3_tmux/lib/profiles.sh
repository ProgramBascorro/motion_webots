apply_profile() {
  local profile="$1"
  case "$profile" in
    webots)
      WEBOTS_CMD="ros2 launch op3_webots_ros2 robot_launch.py"
      MANAGER_CMD="ros2 launch op3_manager op3_simulation.launch.py"
      TELEOP_CMD="ros2 launch op3_joy_teleop op3_joy_teleop.launch.py"
      FOXGLOVE_CMD="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
      YOLO_VISION_CMD="ros2 launch op3_yolo_vision yolo.launch.py"
      LOCALIZATION_CMD="ros2 launch soccer_localization localization.launch.py use_rviz:=false"
      BALL_HEAD_TRACKING_CMD="ros2 launch op3_ball_localization yolo_scan_only.launch.py"
      BALL_LOCALIZER_CMD="ros2 launch op3_ball_localization ball_localizer.launch.py"
      OFFSET_TUNER_CMD="ros2 launch op3_offset_tuner_server op3_offset_tuner_server.launch.xml"
      DEMO_CMD="ros2 launch op3_demo demo_yolo.launch.xml"
      START_DELAY_SEC="4.0"
      ROS_DOMAIN_ID_DEFAULT="0"
      ;;
    real_robot)
      WEBOTS_CMD="ros2 launch op3_webots_ros2 robot_launch.py"
      MANAGER_CMD="ros2 launch op3_bringup op3_manager_with_camera.launch.py"
      TELEOP_CMD="ros2 launch op3_joy_teleop op3_joy_teleop.launch.py"
      FOXGLOVE_CMD="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
      YOLO_VISION_CMD="ros2 launch op3_yolo_vision yolo.launch.py"
      LOCALIZATION_CMD="ros2 launch soccer_localization localization.launch.py use_rviz:=false"
      BALL_HEAD_TRACKING_CMD="ros2 launch op3_ball_localization yolo_scan_only.launch.py"
      BALL_LOCALIZER_CMD="ros2 launch op3_ball_localization ball_localizer.launch.py"
      OFFSET_TUNER_CMD="ros2 launch op3_offset_tuner_server op3_offset_tuner_server.launch.xml"
      DEMO_CMD="ros2 launch op3_demo demo_yolo.launch.xml"
      START_DELAY_SEC="1.0"
      ROS_DOMAIN_ID_DEFAULT="1"
      ;;
    *)
      die "Unknown profile: $profile (use webots or real_robot)"
      ;;
  esac

  ACTION_FILE_SEED="${OP3_ACTION_FILE_SEED:-$WS/src/ROBOTIS-OP3/op3_action_module/data/motion_4095.bin}"
  ACTION_FILE_PATH="${OP3_ACTION_FILE:-$WS/src/ROBOTIS-OP3/op3_action_module/data/motion_custom.bin}"

  # Apply env overrides after profile defaults
  WEBOTS_CMD="${OP3_WEBOTS_CMD:-$WEBOTS_CMD}"
  MANAGER_CMD="${OP3_MANAGER_CMD:-$MANAGER_CMD}"
  TELEOP_CMD="${OP3_TELEOP_CMD:-$TELEOP_CMD}"
  FOXGLOVE_CMD="${OP3_FOXGLOVE_CMD:-$FOXGLOVE_CMD}"
  YOLO_VISION_CMD="${OP3_YOLO_VISION_CMD:-$YOLO_VISION_CMD}"
  LOCALIZATION_CMD="${OP3_LOCALIZATION_CMD:-$LOCALIZATION_CMD}"
  BALL_HEAD_TRACKING_CMD="${OP3_BALL_HEAD_TRACKING_CMD:-$BALL_HEAD_TRACKING_CMD}"
  BALL_LOCALIZER_CMD="${OP3_BALL_LOCALIZER_CMD:-$BALL_LOCALIZER_CMD}"
  OFFSET_TUNER_CMD="${OP3_OFFSET_TUNER_CMD:-$OFFSET_TUNER_CMD}"
  ACTION_EDITOR_CMD="${OP3_ACTION_EDITOR_CMD:-$ACTION_EDITOR_CMD}"
  ACTION_WEB_CMD="${OP3_ACTION_WEB_CMD:-$ACTION_WEB_CMD}"
  DEMO_CMD="${OP3_DEMO_CMD:-$DEMO_CMD}"
  DEMO_MODE="${OP3_DEMO_MODE:-$DEMO_MODE}"
  DEMO_WAIT_STEP_SEC="${OP3_DEMO_WAIT_STEP_SEC:-$DEMO_WAIT_STEP_SEC}"
  DEMO_WAIT_STEPS="${OP3_DEMO_WAIT_STEPS:-$DEMO_WAIT_STEPS}"
  START_DELAY_SEC="${OP3_START_DELAY_SEC:-$START_DELAY_SEC}"
  ROS_DOMAIN_ID_DEFAULT="${ROS_DOMAIN_ID_DEFAULT_ENV:-$ROS_DOMAIN_ID_DEFAULT}"
}
