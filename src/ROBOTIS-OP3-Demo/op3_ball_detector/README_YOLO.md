# YOLO Ball Detector (op3_ball_detector)

A YOLO-based ball detector that is a **drop-in replacement** for the classic
Hough-circle `ball_detector_node`. It runs an [Ultralytics](https://docs.ultralytics.com)
YOLO model on the camera stream and republishes detections in the exact format
the existing `op3_demo` soccer pipeline expects.

## Files

| File | Purpose |
| --- | --- |
| `scripts/yolo_ball_detector.py` | The detector node (Python / rclpy). Loads `.pt`, `.onnx`, **or** an OpenVINO dir. |
| `launch/yolo_ball_detector.launch.py` | Brings up `usb_cam` + the detector (PyTorch `.pt`). |
| `launch/yolo_ball_detector_onnx.launch.py` | Same, but runs an `.onnx` model via onnxruntime. |
| `launch/yolo_ball_detector_openvino.launch.py` | Same, but runs an OpenVINO model (fastest on Intel CPU). |
| `config/yolo_ball_detector_params.yaml` | Tunable parameters for the `.pt` setup (**single source of truth**, incl. `model_path`). |
| `config/yolo_ball_detector_onnx_params.yaml` | Same, for the `.onnx` setup (`model_path` → `model/*.onnx`). |
| `config/yolo_ball_detector_openvino_params.yaml` | Same, for the OpenVINO setup (`model_path` → `model/*_openvino_model/`). |
| `model/*.pt`, `model/*.onnx`, `model/*_openvino_model/` | YOLO weights (class 0 = `Bola-baru`). |

## Topics (under the `ball_detector_node` namespace)

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| sub | `image_in` | `CompressedImage` / `Image` | Camera stream (remapped in launch). |
| sub | `enable` | `std_msgs/Bool` | Pause/resume detection. |
| pub | `circle_set` | `op3_ball_detector_msgs/CircleSetStamped` | `x,y` normalized `[-1,1]`, `z` = radius in **pixels** — same as the C++ detector. |
| pub | `image_out` | `sensor_msgs/Image` | Annotated debug image. |

Because it publishes on `/ball_detector_node/circle_set`, the `op3_demo`
`ball_tracker` / `ball_follower` work unchanged.

## Dependencies

`ultralytics` (and `torch`) have **no rosdep key**. In the project Docker image
they are **already baked in** — the project `Dockerfile` installs CPU `torch`/
`torchvision` plus `requirements_yolo.txt`, so when you build/run via
`./script.sh` nothing extra is needed inside the container.

If you ever run outside that image, install them manually in the runtime
environment (CPU-only example):

```bash
apt-get update && apt-get install -y python3-pip   # if pip is missing
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install -r src/ROBOTIS-OP3-Demo/op3_ball_detector/requirements_yolo.txt
```

> **IMPORTANT:** keep `numpy < 2`. ROS Humble's `cv_bridge` is compiled against
> NumPy 1.x and fails with `_ARRAY_API not found` under NumPy 2.x. The
> `requirements_yolo.txt` pins `numpy<2` and a NumPy-1.x-compatible
> `opencv-python` for this reason.

> **ONNX / OpenVINO:** `requirements_yolo.txt` also installs `onnxruntime` and
> `openvino` (both CPU, both compatible with `numpy<2` — `openvino` declares
> `numpy<2.5,>=1.16.6`). Without them ultralytics auto-pip-installs them
> unpinned on first load — lost on container recreate, and the unpinned install
> can drag NumPy 2 back in. `torch` is still needed even for ONNX/OpenVINO
> inference (ultralytics imports it).

`rclpy`, `cv_bridge` and OpenCV/numpy from ROS are declared in `package.xml`.

In the project image this is already persistent (baked into the `Dockerfile`).
If you installed manually into a running container instead, make it persistent
with `docker commit <container> <new-image>` or add the `pip install` steps to
the `Dockerfile` and rebuild.

## Build

Build the package the usual way (inside the project container):

```bash
colcon build --packages-up-to op3_ball_detector
source install/setup.bash
```

## Run

```bash
# usb_cam + YOLO detector (everything read from the YAML)
ros2 launch op3_ball_detector yolo_ball_detector.launch.py

# Use an already-running camera publishing a raw Image:
ros2 launch op3_ball_detector yolo_ball_detector.launch.py \
    use_usb_cam:=false image_topic:=/usb_cam_node/image_raw use_compressed:=false

# One-off override of the YAML (e.g. GPU + a different weight file):
ros2 launch op3_ball_detector yolo_ball_detector.launch.py \
    device:=cuda:0 model_path:=model/v8n.pt
```

### ONNX model

Use the ONNX launch + ONNX param file instead (runs through onnxruntime,
CPUExecutionProvider):

```bash
# default model from config/yolo_ball_detector_onnx_params.yaml
# (model/v8n.onnx exported at imgsz 320 — ~23 ms/frame on an i5-1340P)
ros2 launch op3_ball_detector yolo_ball_detector_onnx.launch.py
```

It is the same node and publishes the same topics — only the weights/backend
differ. `imgsz` in the ONNX YAML **must** match the size the model was exported
with (the shipped `v8n.onnx` is `320`), or onnxruntime rejects the input shape.
To use a different size, re-export and update `imgsz`:
`yolo export model=model/v8n.pt format=onnx imgsz=NNN`.

### OpenVINO model (fastest on Intel CPU)

Use the OpenVINO launch + param file. The model is a **directory**
(`.xml`+`.bin`), exported from the `.pt`:

```bash
# one-time export (needs the openvino package; see Dependencies)
yolo export model=model/v8n.pt format=openvino imgsz=320
# -> model/v8n_openvino_model/  (already shipped in this repo)

ros2 launch op3_ball_detector yolo_ball_detector_openvino.launch.py
```

On Intel CPUs OpenVINO auto-tunes threading ("LATENCY mode") and is both faster
and more stable than onnxruntime, which oversubscribes all cores. **Safe with
`numpy<2`:** `openvino` declares `numpy<2.5,>=1.16.6`, so it does not pull NumPy 2.

### Performance levers

Rough CPU cost on the i5-1340P (single class, imgsz 320 unless noted):

| Backend / model | ms/frame | notes |
| --- | --- | --- |
| ONNX v8s @ 640 | ≈120 | the old default |
| ONNX v8n @ 640 | ≈74 | |
| ONNX v8n @ 416 | ≈36 | |
| ONNX v8n @ 320 | ≈23–59 | varies — thread oversubscription |
| **OpenVINO v8n @ 320** | **≈26** | fastest + stable (current default backend) |

Biggest levers, in order: **smaller `imgsz`** (re-export — ONNX/OpenVINO input
shape is fixed at export), **`v8n` over `v8s`**, **OpenVINO over onnxruntime on
Intel**, and **cap `detection_rate`** (15 Hz here) so inference does not run
flat-out and starve walking. Further: INT8 export
(`... format=openvino int8=True` with calibration data) for another ~2×.

### Changing the model

Edit `model_path` in the matching param YAML
(`yolo_ball_detector_params.yaml` for `.pt`,
`yolo_ball_detector_onnx_params.yaml` for `.onnx`,
`yolo_ball_detector_openvino_params.yaml` for OpenVINO) and **rebuild**
(`colcon build --packages-select op3_ball_detector`) so the change lands in
`install/`. The value may be an absolute path, or relative to the package share
dir — e.g. `model/v8s.pt` resolves to
`<share>/op3_ball_detector/model/v8s.pt`. The launch args below default to empty
and only override the YAML when you pass them, so the YAML always wins otherwise.

Run the node standalone:

```bash
ros2 run op3_ball_detector yolo_ball_detector.py --ros-args \
    --params-file install/op3_ball_detector/share/op3_ball_detector/config/yolo_ball_detector_params.yaml
```

## Launch arguments

| Arg | Default | Description |
| --- | --- | --- |
| `use_usb_cam` | `true` | Also start the `usb_cam` node. |
| `image_topic` | `/usb_cam_node/image_raw/compressed` | Camera topic mapped to `image_in`. |
| `use_compressed` | `""` (use YAML) | Override only: `true` = `CompressedImage`, `false` = raw `Image`. |
| `model_path` | `""` (use YAML) | Override only: abs path or relative-to-share weights. |
| `device` | `""` (use YAML) | Override only: `cpu` or e.g. `cuda:0`. |

> The last three default to empty = **use the YAML**. They override the YAML
> only when you pass a non-empty value, so editing the YAML always takes effect.

See `config/yolo_ball_detector_params.yaml` for the full parameter list
(confidence/IoU thresholds, `imgsz`, `target_classes`, `detection_rate`, …).
