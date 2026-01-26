#!/bin/bash

# YOLO Performance Testing Script

echo "===== YOLO Vision Performance Testing ====="
echo ""
echo "This script helps you test different performance configurations"
echo ""

# Get the workspace directory
WORKSPACE_DIR="/home/farhan/Projects/Surgical_lokalisasi_bismillah/motion_webots_farhan_coba"
CONFIG_FILE="$WORKSPACE_DIR/src/op3_yolo_vision/config/yolo.yaml"

# Backup original config
if [ ! -f "$CONFIG_FILE.backup" ]; then
    cp "$CONFIG_FILE" "$CONFIG_FILE.backup"
    echo "✓ Backed up original config to yolo.yaml.backup"
fi

# Function to update config
update_config() {
    local input_size=$1
    local debug=$2
    local skip=$3

    cat > "$CONFIG_FILE" <<EOF
yolo_detector:
  ros__parameters:
    image_topic: /robotis_op3/camera/image_raw
    ball_center_topic: /vision/yolo/ball_center
    model_path: models/yolo.onnx
    use_gpu: false
    use_fp16: false
    dnn_backend: opencv
    dnn_target: cpu
    input_size: $input_size
    ball_confidence_threshold: 0.2
    goalpost_confidence_threshold: 0.2
    robot_confidence_threshold: 0.2
    intersection_confidence_threshold: 0.4
    nms_threshold: 0.5
    nms_score_threshold: 0.1
    publish_debug: $debug
    frame_skip: $skip
EOF
}

# Menu
echo "Choose a performance profile:"
echo ""
echo "1) Balanced (416x416, no debug, no skip) - RECOMMENDED"
echo "2) Maximum Performance (320x320, no debug, skip=2)"
echo "3) High Quality (640x640, with debug, no skip)"
echo "4) Custom configuration"
echo "5) Restore original config"
echo ""
read -p "Enter choice [1-5]: " choice

case $choice in
    1)
        echo "Setting: Balanced mode"
        update_config 416 false 1
        ;;
    2)
        echo "Setting: Maximum Performance mode"
        update_config 320 false 2
        ;;
    3)
        echo "Setting: High Quality mode"
        update_config 640 true 1
        ;;
    4)
        read -p "Input size [416]: " size
        size=${size:-416}
        read -p "Enable debug? [false]: " debug
        debug=${debug:-false}
        read -p "Frame skip [1]: " skip
        skip=${skip:-1}
        echo "Setting: Custom ($size, debug=$debug, skip=$skip)"
        update_config $size $debug $skip
        ;;
    5)
        if [ -f "$CONFIG_FILE.backup" ]; then
            cp "$CONFIG_FILE.backup" "$CONFIG_FILE"
            echo "✓ Restored original config"
        else
            echo "✗ No backup found"
        fi
        exit 0
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "✓ Configuration updated!"
echo ""
echo "Current settings:"
grep -E "input_size:|publish_debug:|frame_skip:" "$CONFIG_FILE"
echo ""
echo "To run the node:"
echo "  source install/setup.bash"
echo "  ros2 launch op3_yolo_vision yolo.launch.py"
echo ""
echo "To monitor performance:"
echo "  ros2 topic echo /rosout | grep Performance"
echo ""
