from __future__ import annotations

from pathlib import Path
from typing import Any


class ModelCache:
    def __init__(self) -> None:
        self._cache: dict[str, Any] = {}

    def get_session(self, model_path: str):
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError("onnxruntime is not installed") from exc

        path = str(Path(model_path).resolve())
        if path not in self._cache:
            self._cache[path] = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        return self._cache[path]
