# Bascorro Studio Vision Lab Architecture

## Layers

### 1. Frontend UI layer
- Next.js app in `apps/web`
- React Flow canvas for graph editing
- Toolbar, inspector, preview pane, status bar, and execution log
- Graph save/load and preset loading

### 2. Backend pipeline engine
- FastAPI app in `apps/server/app`
- Graph validation and topological sort
- Synchronous frame-by-frame executor
- WebSocket event stream for previews, metrics, and errors

### 3. I/O adapter layer
- Image file input
- Video file input
- Backend-local webcam input
- Folder replay from uploaded images

## Execution Model
- One frame enters the graph at a time
- Nodes execute in topological order
- Results are cached per frame only
- Each node emits latency, optional FPS estimate, and previewable output
- Model loading is cached across frames through `ModelCache`

## Node System
- Node metadata lives in `apps/server/app/nodes/registry.py`
- The frontend uses `/api/node-types` to build forms and palette entries
- Multi-input nodes like `draw_detections` and `mask_overlay` resolve inputs by upstream output kind

## Preview Transport
- Images and masks are downscaled before WebSocket transport
- Image previews are base64-encoded JPEG or PNG data URLs
- Detection, JSON, and metrics outputs use compact JSON payloads

## Future ROS Adapter Path
The engine deliberately isolates frame ingestion from node execution. To add optional ROS later:
- add `RosImageTopicAdapter` under `apps/server/app/adapters/`
- keep `FramePacket` unchanged
- translate ROS image messages into `FramePacket`
- optionally add ROS output publishers as a separate adapter/output layer without changing node executors
