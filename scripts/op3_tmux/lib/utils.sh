die() { echo "${RED}ERROR:${RST} $*" >&2; exit 1; }

ensure_action_file() {
  local target="${ACTION_FILE_PATH:-}"
  local seed="${ACTION_FILE_SEED:-}"

  if [[ -z "$target" ]]; then
    return 0
  fi
  if [[ -f "$target" ]]; then
    return 0
  fi
  if [[ ! -f "$seed" ]]; then
    echo "${YLW}Warning:${RST} Action seed file not found: $seed"
    return 0
  fi

  mkdir -p "$(dirname "$target")"
  cp "$seed" "$target"
  echo "${DIM}Initialized action file:${RST} $target"
}
