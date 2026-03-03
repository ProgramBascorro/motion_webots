from __future__ import annotations

from fastapi import APIRouter, Request

from app.core.models import GraphValidationResponse, PipelineGraph
from app.graph.validator import validate_graph

router = APIRouter(prefix="/api/graphs", tags=["graphs"])


@router.post("/validate", response_model=GraphValidationResponse)
async def validate_pipeline_graph(graph: PipelineGraph, request: Request) -> GraphValidationResponse:
    registry = request.app.state.registry
    return validate_graph(graph, registry)
