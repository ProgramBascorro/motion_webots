# Camera Recording Tool - Quick Reference

## 📹 What This Tool Does

Records ROS2 camera topics to MP4 video files for:
- YOLO training/testing
- Debugging vision systems
- Recording matches/demos
- Creating datasets

## 🚀 Quick Start

### Easiest Way (Interactive)
```bash
./quick_record.sh
```
Just follow the prompts!

### Direct Command
```bash
./record_camera.py --topic /robotis_op3/camera/image_raw --duration 30
```

## 📁 Files Created

| File | Purpose |
|------|---------|
| `record_camera.py` | Main recording tool (Python) |
| `quick_record.sh` | Interactive wrapper (bash) |
| `test_recorder.sh` | Test dependencies |
| `CAMERA_RECORDING_GUIDE.md` | Full documentation |

## 🎯 Common Commands

```bash
# Record for 30 seconds
./record_camera.py --topic /robotis_op3/camera/image_raw --duration 30

# Record YOLO debug view
./record_camera.py --topic /vision/yolo/debug --output yolo_test.mp4

# Record until Ctrl+C
./record_camera.py --topic /camera/image_raw

# List available topics
./record_camera.py --list

# Record without preview (faster)
./record_camera.py --topic /camera/image --no-preview --duration 60
```

## 🔧 Installation Check

```bash
# Test if everything works
./test_recorder.sh
```

If missing dependencies:
```bash
sudo apt install ros-humble-cv-bridge python3-opencv
```

## 💡 Usage Examples

### Record Match
```bash
# Start before match
./record_camera.py --topic /robotis_op3/camera/image_raw --output match.mp4

# Robot plays soccer...

# Press Ctrl+C when done
```

### Record YOLO Testing
```bash
# Record YOLO detection with bounding boxes
export ROS_DOMAIN_ID=1
./record_camera.py --topic /vision/yolo/debug --duration 60
```

### Extract Frames for Training
```bash
# After recording, extract frames
ffmpeg -i recording.mp4 -vf fps=1 frame_%04d.png
```

## 📊 Preview Window

When recording (if preview enabled):
- **Red dot + "REC"** = Recording
- **Frame count** = Number of frames
- **Time** = Elapsed time
- Press **'q'** = Stop and save
- Press **Ctrl+C** = Stop and save

## 🎓 Tips

1. **Set ROS_DOMAIN_ID** if needed:
   ```bash
   export ROS_DOMAIN_ID=1
   ```

2. **Check topics first**:
   ```bash
   ros2 topic list | grep image
   ```

3. **Test with short duration** first:
   ```bash
   ./record_camera.py --topic /camera/image --duration 5
   ```

4. **Use `--no-preview`** for better performance:
   ```bash
   ./record_camera.py --topic /camera/image --no-preview
   ```

## 📝 Output Files

Default naming: `recording_YYYYMMDD_HHMMSS.mp4`

Example: `recording_20260127_153045.mp4`

## ⚙️ Options

| Option | Description | Example |
|--------|-------------|---------|
| `--topic` | Camera topic | `--topic /camera/image_raw` |
| `--output` | Filename | `--output my_video.mp4` |
| `--duration` | Seconds | `--duration 30` |
| `--fps` | Frame rate | `--fps 15` |
| `--no-preview` | No window | `--no-preview` |
| `--list` | Show topics | `--list` |

## 🐛 Troubleshooting

**No topics found?**
```bash
export ROS_DOMAIN_ID=1
ros2 topic list | grep image
```

**Dependencies missing?**
```bash
./test_recorder.sh
```

**Preview doesn't work?**
```bash
# Use without preview
./record_camera.py --topic /camera/image --no-preview
```

## 📚 Full Documentation

See `CAMERA_RECORDING_GUIDE.md` for complete documentation.

---

**Made for easy YOLO analysis and vision debugging! 🎥**
