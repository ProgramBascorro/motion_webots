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
      BALL_LOCALIZER_CMD="ros2 launch op3_ball_localization ball_localizer.launch.py"
      START_DELAY_SEC="4.0"
      ROS_DOMAIN_ID_DEFAULT="0"
      ;;
    real_robot)
      WEBOTS_CMD="ros2 launch op3_webots_ros2 robot_launch.py"
      MANAGER_CMD="ros2 launch op3_manager op3_simulation.launch.py"
      TELEOP_CMD="ros2 launch op3_joy_teleop op3_joy_teleop.launch.py"
      FOXGLOVE_CMD="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
      YOLO_VISION_CMD="ros2 launch op3_yolo_vision yolo.launch.py"
      LOCALIZATION_CMD="ros2 launch soccer_localization localization.launch.py use_rviz:=false"
      BALL_LOCALIZER_CMD="ros2 launch op3_ball_localization ball_localizer.launch.py"
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
  BALL_LOCALIZER_CMD="${OP3_BALL_LOCALIZER_CMD:-$BALL_LOCALIZER_CMD}"
  ACTION_EDITOR_CMD="${OP3_ACTION_EDITOR_CMD:-$ACTION_EDITOR_CMD}"
  ACTION_WEB_CMD="${OP3_ACTION_WEB_CMD:-$ACTION_WEB_CMD}"
  START_DELAY_SEC="${OP3_START_DELAY_SEC:-$START_DELAY_SEC}"
  ROS_DOMAIN_ID_DEFAULT="${ROS_DOMAIN_ID_DEFAULT_ENV:-$ROS_DOMAIN_ID_DEFAULT}"
}
