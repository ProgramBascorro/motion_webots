#!/usr/bin/env bash

# Quick integration test to verify ttyd is included in install-deps and doctor

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "🔍 Testing ttyd integration in script.sh"
echo "=========================================="
echo ""

# 1. Check if ttyd check is in doctor (health.sh)
echo "1. Checking doctor/health check integration..."
if grep -q "ttyd" "$WS_DIR/scripts/op3_tmux/lib/health.sh"; then
  echo "   ✓ ttyd check found in health.sh"
else
  echo "   ✗ ttyd check NOT found in health.sh"
  exit 1
fi
echo ""

# 2. Check if ttyd install is in install-deps (actions.sh)
echo "2. Checking install-deps integration..."
if grep -q "ttyd" "$WS_DIR/scripts/op3_tmux/lib/actions.sh"; then
  echo "   ✓ ttyd installation found in actions.sh"
else
  echo "   ✗ ttyd installation NOT found in actions.sh"
  exit 1
fi
echo ""

# 3. Check if install_ttyd.sh script exists
echo "3. Checking install script..."
if [[ -f "$WS_DIR/scripts/install_ttyd.sh" ]]; then
  echo "   ✓ install_ttyd.sh exists"
  if [[ -x "$WS_DIR/scripts/install_ttyd.sh" ]]; then
    echo "   ✓ install_ttyd.sh is executable"
  else
    echo "   ✗ install_ttyd.sh is NOT executable"
    exit 1
  fi
else
  echo "   ✗ install_ttyd.sh NOT found"
  exit 1
fi
echo ""

# 4. Check non-interactive mode support
echo "4. Checking non-interactive mode..."
if grep -q "NON_INTERACTIVE" "$WS_DIR/scripts/install_ttyd.sh"; then
  echo "   ✓ Non-interactive mode supported"
else
  echo "   ✗ Non-interactive mode NOT supported"
  exit 1
fi
echo ""

echo "=========================================="
echo "✅ All integration tests passed!"
echo ""
echo "The following integrations are active:"
echo "  • ./script.sh → 'Health check' → checks ttyd"
echo "  • ./script.sh → 'Install deps' → installs ttyd"
echo "  • ./scripts/install_ttyd.sh → standalone install"
echo ""
echo "Next steps:"
echo "  1. Run: ./script.sh"
echo "  2. Select: 'Health check' (will check ttyd)"
echo "  3. Or select: 'Install deps' (will install ttyd)"
echo ""
