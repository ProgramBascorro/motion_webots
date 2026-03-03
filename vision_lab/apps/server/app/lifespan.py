from __future__ import annotations

from contextlib import asynccontextmanager

from fastapi import FastAPI

from app.core.session_manager import SessionManager
from app.engine.context import ExecutionContext
from app.engine.executor import PipelineExecutor
from app.inference.detection import DetectionModelRunner
from app.inference.model_cache import ModelCache
from app.inference.segmentation import SegmentationModelRunner
from app.nodes.registry import NodeRegistry
from app.utils.file_store import FileStore


@asynccontextmanager
async def lifespan(app: FastAPI):
    registry = NodeRegistry()
    sessions = SessionManager()
    file_store = FileStore()
    model_cache = ModelCache()
    detection_runner = DetectionModelRunner(model_cache)
    segmentation_runner = SegmentationModelRunner(model_cache)
    context = ExecutionContext(
        registry=registry,
        sessions=sessions,
        file_store=file_store,
        detection_runner=detection_runner,
        segmentation_runner=segmentation_runner,
    )
    app.state.registry = registry
    app.state.sessions = sessions
    app.state.file_store = file_store
    app.state.executor = PipelineExecutor(context)
    yield
