#!/bin/bash
# Quick record wrapper - simplified interface for common use cases

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "================================================"
echo "  ROS2 Camera Recorder - Quick Mode"
echo "================================================"
echo ""

# Common camera topics
echo "Common camera topics:"
echo "  1) /robotis_op3/camera/image_raw  (OP3 camera)"
echo "  2) /vision/yolo/debug             (YOLO debug view)"
echo "  3) /usb_cam/image_raw             (USB camera)"
echo "  4) Custom topic"
echo "  5) List all topics"
echo ""

read -p "Select option [1]: " choice
choice=${choice:-1}

case $choice in
    1)
        TOPIC="/robotis_op3/camera/image_raw"
        ;;
    2)
        TOPIC="/vision/yolo/debug"
        ;;
    3)
        TOPIC="/usb_cam/image_raw"
        ;;
    4)
        read -p "Enter topic name: " TOPIC
        ;;
    5)
        export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
        echo ""
        echo "Available image topics:"
        ros2 topic list | grep -E "(image|camera)" | nl
        echo ""
        read -p "Enter topic name: " TOPIC
        ;;
    *)
        echo "Invalid choice, using default"
        TOPIC="/robotis_op3/camera/image_raw"
        ;;
esac

echo ""
read -p "Duration in seconds (or press Enter for manual stop): " DURATION
echo ""
read -p "Output filename (or press Enter for auto-generated): " OUTPUT
echo ""

# Build command
CMD="python3 $SCRIPT_DIR/record_camera.py --topic $TOPIC"

if [ -n "$DURATION" ]; then
    CMD="$CMD --duration $DURATION"
fi

if [ -n "$OUTPUT" ]; then
    CMD="$CMD --output $OUTPUT"
fi

echo "================================================"
echo "Starting recording..."
echo "Command: $CMD"
echo "================================================"
echo ""
echo "Press Ctrl+C or 'q' in preview window to stop"
echo ""

# Set ROS_DOMAIN_ID if not set
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# Run the recorder
$CMD
