#!/usr/bin/env bash

# Allow running via `zsh scripts/op3_tmux/main.sh` (or any non-bash shell) by re-executing under bash.
if [ -z "${BASH_VERSION-}" ]; then
  if [ -n "${ZSH_VERSION-}" ]; then
    case "${ZSH_EVAL_CONTEXT-}" in
      *:file)
        echo "ERROR: Source this from bash (via ./script.sh), or execute it directly under bash." >&2
        return 1
        ;;
    esac
  fi
  if command -v bash >/dev/null 2>&1; then
    exec bash "$0" "$@"
  fi
  echo "ERROR: bash is required to run this script." >&2
  exit 1
fi

set -euo pipefail

OP3_TMUX_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OP3_TMUX_LIB="$OP3_TMUX_ROOT/lib"

source "$OP3_TMUX_LIB/colors.sh"
source "$OP3_TMUX_LIB/utils.sh"
source "$OP3_TMUX_LIB/banner.sh"
source "$OP3_TMUX_LIB/defaults.sh"
source "$OP3_TMUX_LIB/profiles.sh"
source "$OP3_TMUX_LIB/health.sh"
source "$OP3_TMUX_LIB/commands.sh"
source "$OP3_TMUX_LIB/tmux.sh"
source "$OP3_TMUX_LIB/actions.sh"
source "$OP3_TMUX_LIB/selection.sh"
source "$OP3_TMUX_LIB/start.sh"
source "$OP3_TMUX_LIB/menu.sh"
source "$OP3_TMUX_LIB/args.sh"

op3_tmux_main() {
  op3_tmux_dispatch "$@"
}
