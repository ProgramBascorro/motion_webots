from __future__ import annotations

from pathlib import Path


APP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = APP_ROOT.parents[1]
RUNTIME_ROOT = WORKSPACE_ROOT / ".runtime"
UPLOAD_ROOT = RUNTIME_ROOT / "uploads"
MODEL_ROOT = APP_ROOT / "models"
DEFAULT_DETECTION_MODEL = WORKSPACE_ROOT.parent / "src" / "op3_yolo_vision" / "models" / "yolo.onnx"
