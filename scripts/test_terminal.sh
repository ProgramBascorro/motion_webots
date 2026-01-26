#!/usr/bin/env bash

# Quick test script for web terminal functionality
# Run this after building the workspace

set -euo pipefail

echo "🔍 Testing Bascorro Studio Web Terminal Setup"
echo "=============================================="
echo ""

# 1. Check ttyd installation
echo "1. Checking ttyd installation..."
if command -v ttyd >/dev/null 2>&1; then
  version=$(ttyd --version 2>&1 | head -n1)
  echo "   ✓ ttyd installed: $version"
else
  echo "   ✗ ttyd not found"
  echo "   Run: ./scripts/install_ttyd.sh"
  exit 1
fi
echo ""

# 2. Check ROS2 workspace
echo "2. Checking ROS2 workspace build..."
if [[ -f "install/setup.bash" ]]; then
  echo "   ✓ Workspace is built"
  source install/setup.bash
else
  echo "   ✗ Workspace not built"
  echo "   Run: colcon build"
  exit 1
fi
echo ""

# 3. Check if terminal_server executable exists
echo "3. Checking terminal_server executable..."
if ros2 pkg executables bascorro_studio | grep -q "terminal_server"; then
  echo "   ✓ terminal_server found in bascorro_studio package"
else
  echo "   ✗ terminal_server not found"
  echo "   Rebuild workspace: colcon build --packages-select bascorro_studio"
  exit 1
fi
echo ""

# 4. Check web UI files
echo "4. Checking web UI updates..."
if grep -q "renderTerminal" src/bascorro_studio/web/src/App.jsx; then
  echo "   ✓ Terminal tab added to web UI"
else
  echo "   ✗ Terminal tab not found in App.jsx"
  exit 1
fi
echo ""

# 5. Check launch file
echo "5. Checking launch file..."
if grep -q "terminal_server" src/bascorro_studio/launch/bascorro_studio.launch.py; then
  echo "   ✓ terminal_server added to launch file"
else
  echo "   ✗ terminal_server not in launch file"
  exit 1
fi
echo ""

# 6. Test terminal server startup (dry run)
echo "6. Testing terminal_server node (5 second test)..."
timeout 5 ros2 run bascorro_studio terminal_server >/dev/null 2>&1 &
pid=$!
sleep 2

if kill -0 $pid 2>/dev/null; then
  echo "   ✓ terminal_server node starts successfully"
  kill $pid 2>/dev/null || true
  wait $pid 2>/dev/null || true
else
  echo "   ✗ terminal_server failed to start"
  exit 1
fi
echo ""

# 7. Summary
echo "=============================================="
echo "✅ All checks passed!"
echo ""
echo "Next steps:"
echo "  1. Launch studio: ./scripts/bascorro_studio.sh"
echo "  2. Open browser: http://localhost:5173"
echo "  3. Click 'Terminal' tab in sidebar"
echo "  4. Or direct access: http://localhost:7681"
echo ""
echo "Optional - Enable authentication:"
echo "  export OP3_STUDIO_TERMINAL_CREDENTIAL=\"user:password\""
echo "  ./scripts/bascorro_studio.sh"
echo ""
