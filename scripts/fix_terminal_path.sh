#!/usr/bin/env bash

# Quick fix: Add ~/.local/bin to PATH and restart terminal_server
# This is needed after installing ttyd for the first time

set -euo pipefail

echo "🔧 Fixing terminal_server PATH issue..."
echo ""

# 1. Check if ttyd is installed
if [[ ! -f "$HOME/.local/bin/ttyd" ]] && ! command -v ttyd >/dev/null 2>&1; then
  echo "❌ ttyd is not installed!"
  echo "   Run: ./scripts/install_ttyd.sh"
  exit 1
fi

# 2. Add to bashrc if not already there
BASHRC="$HOME/.bashrc"
if ! grep -q "/.local/bin" "$BASHRC" 2>/dev/null; then
  echo "📝 Adding ~/.local/bin to PATH in $BASHRC"
  echo '' >> "$BASHRC"
  echo '# Added by bascorro_studio terminal setup' >> "$BASHRC"
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$BASHRC"
  echo "   ✓ PATH updated in $BASHRC"
else
  echo "   ✓ PATH already configured in $BASHRC"
fi

# 3. Check if running in tmux
if [[ -n "${TMUX:-}" ]] || tmux list-sessions 2>/dev/null | grep -q "op3"; then
  echo ""
  echo "🔄 Terminal server needs to be restarted..."
  echo ""
  echo "Option 1 (Recommended): Restart the entire session"
  echo "  1. Exit bascorro_studio (Ctrl+C or close terminal)"
  echo "  2. Restart: ./scripts/bascorro_studio.sh"
  echo ""
  echo "Option 2: Just restart terminal_server node"
  echo "  Run in a new terminal:"
  echo "    export PATH=\"\$HOME/.local/bin:\$PATH\""
  echo "    ros2 run bascorro_studio terminal_server"
  echo ""
else
  echo ""
  echo "✅ Setup complete! Now you can start bascorro_studio:"
  echo "   ./scripts/bascorro_studio.sh"
  echo ""
fi

echo "After restart, terminal will be available at:"
echo "  • http://localhost:7681 (direct)"
echo "  • http://localhost:5173 (Web UI → Terminal tab)"
echo ""
