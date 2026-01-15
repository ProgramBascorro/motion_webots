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
      START_DELAY_SEC="1.0"
      ROS_DOMAIN_ID_DEFAULT="1"
      ;;
    *)
      die "Unknown profile: $profile (use webots or real_robot)"
      ;;
  esac

  # Apply env overrides after profile defaults
  WEBOTS_CMD="${OP3_WEBOTS_CMD:-$WEBOTS_CMD}"
  MANAGER_CMD="${OP3_MANAGER_CMD:-$MANAGER_CMD}"
  TELEOP_CMD="${OP3_TELEOP_CMD:-$TELEOP_CMD}"
  FOXGLOVE_CMD="${OP3_FOXGLOVE_CMD:-$FOXGLOVE_CMD}"
  YOLO_VISION_CMD="${OP3_YOLO_VISION_CMD:-$YOLO_VISION_CMD}"
  LOCALIZATION_CMD="${OP3_LOCALIZATION_CMD:-$LOCALIZATION_CMD}"
  START_DELAY_SEC="${OP3_START_DELAY_SEC:-$START_DELAY_SEC}"
  ROS_DOMAIN_ID_DEFAULT="${ROS_DOMAIN_ID_DEFAULT_ENV:-$ROS_DOMAIN_ID_DEFAULT}"
}
