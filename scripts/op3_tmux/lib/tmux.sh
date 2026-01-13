session_exists() { tmux has-session -t "$SESSION" 2>/dev/null; }

tmux_send() {
  local target="$1"
  local cmd="$2"
  tmux send-keys -t "$target" "$SHELL_RUNNER \"$(printf "%q" "$cmd")\"" C-m
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
