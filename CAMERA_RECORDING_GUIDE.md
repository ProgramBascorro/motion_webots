# Camera Recording Tool for YOLO Analysis

This tool records ROS2 camera topics to MP4 video files for later analysis with YOLO or debugging.

## Quick Start

### Method 1: Quick Interactive Script (Easiest)
```bash
./quick_record.sh
```
Just answer the prompts!

### Method 2: Direct Python Script (More Options)
```bash
# Record OP3 camera for 30 seconds
./record_camera.py --topic /robotis_op3/camera/image_raw --duration 30

# Record YOLO debug view until Ctrl+C
./record_camera.py --topic /vision/yolo/debug

# Record with custom filename
./record_camera.py --topic /camera/image_raw --output my_test.mp4
```

## Common Use Cases

### 1. Record YOLO Detection for Analysis
```bash
# Start YOLO first
export ROS_DOMAIN_ID=1
ros2 launch op3_yolo_vision yolo.launch.py

# Record debug view with bounding boxes
./record_camera.py --topic /vision/yolo/debug --duration 60 --output yolo_test.mp4
```

### 2. Record Raw Camera for Training Data
```bash
# Record raw camera feed
./record_camera.py --topic /robotis_op3/camera/image_raw --duration 120 --output training_data.mp4
```

### 3. Record Full Match/Demo
```bash
# Record entire soccer match
./record_camera.py --topic /robotis_op3/camera/image_raw --output match_20260127.mp4
# Press Ctrl+C when done
```

### 4. Record for Debugging Ball Detection
```bash
# Record ball detector output
./record_camera.py --topic /ball_detector_node/image --duration 30
```

## Command Line Options

```
./record_camera.py [OPTIONS]

Options:
  --topic, -t TOPIC      Camera topic to record (required)
  --output, -o FILE      Output MP4 filename (default: auto-generated)
  --duration, -d SEC     Recording duration in seconds (default: until Ctrl+C)
  --fps FPS              Video FPS (default: 30)
  --no-preview           Disable preview window
  --list, -l             List available image topics and exit

Examples:
  ./record_camera.py --topic /camera/image_raw --duration 30
  ./record_camera.py --topic /robotis_op3/camera/image_raw --output match.mp4
  ./record_camera.py --list
  ./record_camera.py --topic /vision/yolo/debug --fps 15 --no-preview
```

## Available Camera Topics

Common topics (use `--list` to see all):

| Topic | Description |
|-------|-------------|
| `/robotis_op3/camera/image_raw` | OP3 robot camera (raw feed) |
| `/vision/yolo/debug` | YOLO detection with bounding boxes |
| `/vision/yolo/image` | YOLO processed image |
| `/usb_cam/image_raw` | USB camera (if using external cam) |
| `/ball_detector_node/image` | Ball detector output |

## Preview Window Controls

When preview is enabled (default):
- **Red circle + "REC"** = Recording indicator
- **Frame count** = Number of frames recorded
- **Time** = Elapsed recording time
- **Press 'q'** = Stop recording and save
- **Press Ctrl+C** = Stop recording and save

## Output Files

Default filename format: `recording_YYYYMMDD_HHMMSS.mp4`

Example: `recording_20260127_153045.mp4`

Custom filename:
```bash
./record_camera.py --topic /camera/image --output my_video.mp4
```

## Tips for YOLO Analysis

### Recording Tips
1. **Good lighting** - Record in well-lit conditions
2. **Stable camera** - Less motion = better for analysis
3. **Multiple angles** - Record from different positions
4. **Variety** - Include different ball positions/distances

### After Recording
1. **Extract frames** for YOLO training:
```bash
# Extract every 10th frame
ffmpeg -i recording.mp4 -vf "select=not(mod(n\,10))" -vsync vfr frame_%04d.png

# Extract 1 frame per second
ffmpeg -i recording.mp4 -vf fps=1 frame_%04d.png
```

2. **Convert to different format** if needed:
```bash
# Convert to AVI
ffmpeg -i recording.mp4 output.avi

# Compress for sharing
ffmpeg -i recording.mp4 -vcodec libx264 -crf 28 compressed.mp4
```

3. **Analyze with YOLO** offline:
```python
# Example YOLO analysis script
import cv2
from ultralytics import YOLO

model = YOLO('yolo11n.pt')
cap = cv2.VideoCapture('recording.mp4')

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    results = model(frame)
    # Process results...
```

## Troubleshooting

### "No image topics found"
- Check ROS_DOMAIN_ID: `export ROS_DOMAIN_ID=1`
- Ensure camera is running: `ros2 topic list | grep image`
- Start camera node first

### "ImportError: No module named cv_bridge"
- Install ROS2 cv_bridge: `sudo apt install ros-humble-cv-bridge`
- Or use conda/pip: `pip install opencv-python`

### Video file is empty or corrupted
- Ensure you received at least one frame before stopping
- Check topic is publishing: `ros2 topic hz /your/topic`
- Try with preview enabled to see if frames are coming

### Preview window doesn't show
- Check X11 forwarding if using SSH
- Use `--no-preview` for headless recording
- Install OpenCV GUI support: `pip install opencv-contrib-python`

### Recording is laggy/dropping frames
- Reduce FPS: `--fps 15`
- Use `--no-preview` to reduce CPU usage
- Check disk space and write speed

## Performance

| Setting | CPU Usage | Disk Usage (per minute) |
|---------|-----------|------------------------|
| 640x480 @ 30fps | ~20% | ~50 MB |
| 1280x720 @ 30fps | ~35% | ~120 MB |
| With preview | +10% | Same |
| No preview | -10% | Same |

## Integration with Your Workflow

### Before Match/Testing
```bash
# Start recording before running demo
./record_camera.py --topic /robotis_op3/camera/image_raw --output test_run.mp4 &
RECORD_PID=$!

# Run your demo
ros2 launch op3_demo demo.launch.xml

# Stop recording when done
kill -INT $RECORD_PID
```

### Automated Recording
```bash
# Record for exactly 2 minutes during demo
./record_camera.py --topic /robotis_op3/camera/image_raw --duration 120 --output demo_$(date +%H%M%S).mp4
```

### Multiple Camera Recording
```bash
# Terminal 1: Record OP3 camera
./record_camera.py --topic /robotis_op3/camera/image_raw --output cam1.mp4 --no-preview &

# Terminal 2: Record YOLO debug
./record_camera.py --topic /vision/yolo/debug --output yolo.mp4 --no-preview &

# Both record simultaneously!
```

## File Locations

| File | Purpose |
|------|---------|
| `record_camera.py` | Main Python recording script |
| `quick_record.sh` | Interactive bash wrapper |
| `recording_*.mp4` | Default output videos |

## Advanced Usage

### Change Video Codec
Edit `record_camera.py` line with `fourcc`:
```python
# H.264 codec (better compression)
fourcc = cv2.VideoWriter_fourcc(*'H264')

# MJPEG (faster encoding)
fourcc = cv2.VideoWriter_fourcc(*'MJPG')

# Default MP4V
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
```

### Add Timestamp Overlay
Edit the `image_callback` method to add custom overlays:
```python
# Add timestamp
from datetime import datetime
timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
cv2.putText(preview, timestamp, (10, height - 70),
           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
```

## Summary

**Quick Recording:**
```bash
./quick_record.sh
```

**Common Command:**
```bash
./record_camera.py --topic /robotis_op3/camera/image_raw --duration 60
```

**For YOLO Analysis:**
```bash
./record_camera.py --topic /vision/yolo/debug --output yolo_analysis.mp4
```

**List Topics:**
```bash
./record_camera.py --list
```

Happy recording! 🎥
