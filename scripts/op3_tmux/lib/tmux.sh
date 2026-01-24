session_exists() { tmux has-session -t "$SESSION" 2>/dev/null; }

shell_quote_single() {
  local s="$1"
  printf "'"
  local i
  for ((i=0; i<${#s}; i++)); do
    if [[ "${s:i:1}" == "'" ]]; then
      printf "'\\''"
    else
      printf "%s" "${s:i:1}"
    fi
  done
  printf "'"
}

tmux_send() {
  local target="$1"
  local cmd="$2"
  local quoted_cmd
  quoted_cmd="$(shell_quote_single "$cmd")"
  tmux send-keys -t "$target" "$SHELL_RUNNER $quoted_cmd" C-m
}

tmux_env_set() {
  local key="$1"
  local val="$2"
  tmux set-environment -t "$SESSION" "$key" "$val"
}

tmux_env_get() {
  local key="$1"
  tmux show-environment -t "$SESSION" "$key" 2>/dev/null | sed -n "s/^${key}=//p"
}
