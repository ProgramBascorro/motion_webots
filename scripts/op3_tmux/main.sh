#!/usr/bin/env bash
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
