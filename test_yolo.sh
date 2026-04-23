#!/bin/bash

set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$WORKSPACE"
source install/setup.bash

echo "========================================="
echo "YOLO Detector Test with OpenVINO"
echo "========================================="
echo ""

# Check if binary exists
BINARY="$WORKSPACE/install/op3_yolo_vision/lib/op3_yolo_vision/yolo_detector"
if [ ! -f "$BINARY" ]; then
    echo "✗ Binary not found: $BINARY"
    exit 1
fi

echo "✓ Binary found: $BINARY"
echo ""

# Check library dependencies
echo "Checking OpenVINO and OpenCV libraries:"
ldd "$BINARY" | grep -E "openvino|opencv" | head -10 || true

# Count missing libraries
MISSING=$(ldd "$BINARY" 2>&1 | grep "not found" | wc -l)
if [ $MISSING -gt 0 ]; then
    echo ""
    echo "✗ $MISSING libraries not found!"
    echo "Missing libraries:"
    ldd "$BINARY" 2>&1 | grep "not found"
    exit 1
fi

echo ""
echo "✓ All runtime libraries found"
echo ""
echo "Launching YOLO detector..."
echo "Press Ctrl+C to stop"
echo ""

ros2 launch op3_yolo_vision yolo.launch.py
