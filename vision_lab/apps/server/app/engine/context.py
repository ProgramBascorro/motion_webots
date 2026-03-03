from __future__ import annotations

from dataclasses import dataclass

from app.core.preview import build_preview
from app.core.session_manager import SessionManager
from app.inference.detection import DetectionModelRunner
from app.inference.segmentation import SegmentationModelRunner
from app.nodes.registry import NodeRegistry
from app.utils.file_store import FileStore


@dataclass
class ExecutionContext:
    registry: NodeRegistry
    sessions: SessionManager
    file_store: FileStore
    detection_runner: DetectionModelRunner
    segmentation_runner: SegmentationModelRunner

    def make_preview(self, output_kind: str, payload, preview_max_width: int):
        return build_preview(output_kind, payload, preview_max_width)
