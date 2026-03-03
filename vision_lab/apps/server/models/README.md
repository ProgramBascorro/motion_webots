# Models Directory

Place ONNX detection and segmentation models here for local experiments.

Notes:
- Detection presets default to mock mode, so the app runs even without real models.
- If you disable `use_mock` on an inference node, set `model_path` to an absolute path or a path valid from the server process.
- A convenience detection model may already exist in the wider repo at `src/op3_yolo_vision/models/yolo.onnx`.
