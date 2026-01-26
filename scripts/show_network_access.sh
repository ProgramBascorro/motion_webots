#!/usr/bin/env bash

# Show network access information for bascorro_studio
# Use this to find the URL to access from your phone/other devices

set -euo pipefail

echo "🌐 Bascorro Studio Network Access Information"
echo "=============================================="
echo ""

# Get local IP addresses
echo "📍 Network IP Addresses:"
echo ""

# Try to get the main network interface IP
if command -v ip >/dev/null 2>&1; then
  # Get all non-loopback IPv4 addresses
  ips=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1')

  if [[ -n "$ips" ]]; then
    while IFS= read -r ip; do
      echo "  • $ip"
    done <<< "$ips"
  else
    echo "  ⚠ No network IP found"
  fi
elif command -v hostname >/dev/null 2>&1; then
  # Fallback to hostname
  ip=$(hostname -I 2>/dev/null | awk '{print $1}')
  if [[ -n "$ip" ]]; then
    echo "  • $ip"
  else
    echo "  ⚠ Could not determine IP"
  fi
else
  echo "  ⚠ Could not determine IP (install 'ip' or 'hostname' command)"
fi

echo ""
echo "🔗 Access URLs:"
echo ""

# Get the primary IP
primary_ip=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | head -n1)

if [[ -z "$primary_ip" ]]; then
  primary_ip=$(hostname -I 2>/dev/null | awk '{print $1}')
fi

if [[ -n "$primary_ip" ]]; then
  echo "📱 From Phone/Tablet/Other Device:"
  echo "  Web UI:     http://${primary_ip}:5173"
  echo "  Terminal:   http://${primary_ip}:7681"
  echo "  ROSBridge:  ws://${primary_ip}:9090"
  echo ""
  echo "💻 From This Computer:"
  echo "  Web UI:     http://localhost:5173"
  echo "  Terminal:   http://localhost:7681"
  echo "  ROSBridge:  ws://localhost:9090"
else
  echo "⚠ Could not determine primary IP address"
  echo ""
  echo "💻 From This Computer:"
  echo "  Web UI:     http://localhost:5173"
  echo "  Terminal:   http://localhost:7681"
fi

echo ""
echo "📝 Notes:"
echo "  • Make sure your phone/device is on the same network (WiFi)"
echo "  • Check firewall settings if connection fails"
echo "  • Terminal iframe will auto-detect the correct URL"
echo ""

# Check if services are running
echo "🔍 Service Status:"
echo ""

if pgrep -f "vite.*--port 5173" >/dev/null 2>&1; then
  echo "  ✓ Web UI (Vite) is running"
else
  echo "  ✗ Web UI (Vite) is NOT running"
fi

if pgrep -f "ttyd.*--port 7681" >/dev/null 2>&1; then
  echo "  ✓ Terminal server (ttyd) is running"
else
  echo "  ✗ Terminal server (ttyd) is NOT running"
fi

if pgrep -f "rosbridge" >/dev/null 2>&1; then
  echo "  ✓ ROSBridge is running"
else
  echo "  ✗ ROSBridge is NOT running"
fi

echo ""

# Test if ports are accessible
echo "🔌 Port Accessibility:"
echo ""

for port in 5173 7681 9090; do
  if nc -z -w1 127.0.0.1 $port 2>/dev/null; then
    echo "  ✓ Port $port is accessible"
  else
    echo "  ✗ Port $port is NOT accessible"
  fi
done

echo ""
echo "💡 Quick QR Code (if qrencode installed):"
if command -v qrencode >/dev/null 2>&1 && [[ -n "$primary_ip" ]]; then
  echo ""
  echo "Scan this QR code with your phone:"
  echo ""
  qrencode -t ANSIUTF8 "http://${primary_ip}:5173"
  echo ""
  echo "URL: http://${primary_ip}:5173"
else
  if [[ -n "$primary_ip" ]]; then
    echo "  Install qrencode for QR code: sudo apt install qrencode"
    echo "  Or manually enter: http://${primary_ip}:5173"
  fi
fi

echo ""
echo "✅ Done! Access from your phone using the URLs above."
echo ""
