#!/bin/bash
# Test script for camera recorder

echo "Testing Camera Recorder Tool"
echo "============================="
echo ""

# Check if Python script exists
if [ ! -f "record_camera.py" ]; then
    echo "❌ ERROR: record_camera.py not found"
    exit 1
fi

# Check if executable
if [ ! -x "record_camera.py" ]; then
    echo "⚠️  Making record_camera.py executable..."
    chmod +x record_camera.py
fi

# Check Python dependencies
echo "Checking dependencies..."
python3 -c "import rclpy; import cv2; from cv_bridge import CvBridge" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Missing dependencies!"
    echo ""
    echo "Please install:"
    echo "  sudo apt install ros-humble-cv-bridge python3-opencv"
    echo "  or"
    echo "  pip install opencv-python"
    exit 1
fi
echo "✅ Dependencies OK"
echo ""

# Show help
echo "Testing --help..."
python3 ./record_camera.py --help
echo ""

# List topics (requires ROS running)
echo "Testing --list..."
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
python3 ./record_camera.py --list

echo ""
echo "============================="
echo "✅ Tool is ready to use!"
echo ""
echo "Quick start:"
echo "  ./quick_record.sh"
echo ""
echo "Or directly:"
echo "  ./record_camera.py --topic /robotis_op3/camera/image_raw --duration 10"
echo ""
