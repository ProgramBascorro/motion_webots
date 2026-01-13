run_in_shell() {
  local cmd="$1"
  $SHELL_RUNNER "$cmd"
}

# Build a command that sources setup, exports ROS_DOMAIN_ID, then runs the cmd.
wrap_cmd() {
  local cmd="$1"
  # shellcheck disable=SC2016
  echo "source '$SETUP'; export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-$ROS_DOMAIN_ID_DEFAULT}; $cmd"
}
