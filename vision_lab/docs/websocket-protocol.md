# WebSocket Protocol

Connect to:

```text
WS /ws/run/{session_id}
```

## Messages

### `run_started`
Sent once execution begins.

### `run_progress`
Sent after each frame completes with the current frame index and pipeline FPS estimate.

### `node_preview`
Carries the latest previewable payload for a node.

Fields:
- `session_id`
- `frame_id`
- `node_id`
- `preview_type`
- `preview_payload`

### `node_metrics`
Carries per-node latency and FPS estimate.

### `node_error`
Carries node-specific execution failures or validation issues detected during a run.

### `run_finished`
Sent on normal completion with total frames, total latency, average FPS, and a summary object.

### `run_stopped`
Sent when a user stops the run.

### `run_log`
Optional informational log entry.
