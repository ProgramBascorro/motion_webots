# Bascorro Studio Vision Lab

Bascorro Studio Vision Lab is a local-first MVP for visually debugging computer vision inference pipelines for robotics experiments without requiring ROS in v1.

## What it does
- Build DAG pipelines in the browser with React Flow
- Configure node parameters from an inspector panel
- Run pipelines on image, video, webcam, or folder replay inputs
- Inspect per-node previews, latency, and FPS
- Save and load pipeline graphs as JSON
- Start in mock inference mode when no ONNX models are available

## Workspace Layout
```text
vision_lab/
  apps/web      Next.js frontend
  apps/server   FastAPI backend
  packages/shared
  presets/
  docs/
```

## Setup

### 1. Backend
```bash
cd /home/bascorro/motion_webots/vision_lab/apps/server
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
```

### 2. Frontend
```bash
cd /home/bascorro/motion_webots/vision_lab
pnpm install
```

## Run

### Backend
```bash
cd /home/bascorro/motion_webots/vision_lab/apps/server
source .venv/bin/activate
python3 -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### Frontend
```bash
cd /home/bascorro/motion_webots/vision_lab
pnpm --filter @vision-lab/web dev
```

Open `http://localhost:3000`.

### Launcher Integration
You can also start Vision Lab from the existing workspace launcher:

```bash
cd /home/bascorro/motion_webots
./script.sh --vision-lab
```

Or open `./script.sh`, choose `Launch picker`, then select `Vision Lab`.

## Basic Workflow
1. Load a preset or add nodes from the left palette.
2. Upload image, video, or folder assets with `Upload Assets`.
3. Copy uploaded `file_id` values into the matching input node params.
   The `Image / MP4 Input` node can now consume either a still image upload or an uploaded `.mp4`.
4. Click `Run`.
5. Select any node to inspect its latest preview and metrics.
6. Save the graph JSON when the pipeline looks right.

## API Surface
- `GET /api/health`
- `GET /api/node-types`
- `POST /api/graphs/validate`
- `POST /api/upload`
- `POST /api/run/start`
- `POST /api/run/stop`
- `WS /ws/run/{session_id}`

## Verification
Backend tests:

```bash
cd /home/bascorro/motion_webots/vision_lab
python3 -m pytest apps/server/tests -q
```

## Known Limitations
- Real ONNX support is intentionally narrow and model-specific.
- Webcam input is backend-local only in v1.
- Folder replay uses uploaded images rather than direct browser folder handles.
- The frontend expects uploaded `file_id` values to be pasted into input node params.
- No video export or persistent storage is included in v1.
- Frontend install/build was not executed in this environment because dependencies are not preinstalled in the new workspace.

## ROS Later
Recommended next steps for optional ROS integration:
1. Add a `RosImageTopicAdapter` that converts ROS images to `FramePacket`.
2. Add output adapters for ROS topic publishing of detections, masks, or annotated images.
3. Keep ROS-specific configuration outside node executors so the graph model stays transport-agnostic.
