run_in_shell() {
  local cmd="$1"
  $SHELL_RUNNER "$cmd"
}

# Build a command that sources setup, exports ROS_DOMAIN_ID, then runs the cmd.
wrap_cmd() {
  local cmd="$1"
  local env_script="${OP3_ENV_SCRIPT:-$WS/scripts/op3_env.sh}"
  # shellcheck disable=SC2016
  if [[ -f "$env_script" ]]; then
    echo "source '$env_script'; source '$SETUP'; export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-$ROS_DOMAIN_ID_DEFAULT}; export OP3_ACTION_FILE='${ACTION_FILE_PATH}'; $cmd"
  else
    echo "source '$SETUP'; export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-$ROS_DOMAIN_ID_DEFAULT}; export OP3_ACTION_FILE='${ACTION_FILE_PATH}'; $cmd"
  fi
}
