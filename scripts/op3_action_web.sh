#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[action_web] Deprecated: use bascorro_studio.sh instead."
exec "$SCRIPT_DIR/bascorro_studio.sh" "$@"
