#!/bin/bash

# Test script to verify YOLO works with OpenCV 4.8.1

WORKSPACE="/home/farhan/Projects/Surgical_lokalisasi_bismillah/motion_webots_farhan_coba"
OPENCV_LIB="/home/farhan/Projects/Surgical_lokalisasi_bismillah/third_party/opencv-4.8.1/install/lib"

export LD_LIBRARY_PATH="${OPENCV_LIB}:$LD_LIBRARY_PATH"

cd "$WORKSPACE"
source install/setup.bash

echo "========================================="
echo "YOLO Detector Test with OpenCV 4.8.1"
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
echo "Checking OpenCV libraries:"
ldd "$BINARY" | grep opencv | head -5

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
echo "✓ All OpenCV libraries found"
echo ""
echo "Launching YOLO detector..."
echo "Press Ctrl+C to stop"
echo ""

ros2 launch op3_yolo_vision yolo.launch.py
