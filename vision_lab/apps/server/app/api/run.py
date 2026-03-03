from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request

from app.core.models import RunStartRequest, RunStartResponse, RunStopRequest, RunStopResponse
from app.graph.validator import validate_graph

router = APIRouter(prefix="/api/run", tags=["run"])


@router.post("/start", response_model=RunStartResponse)
async def start_run(payload: RunStartRequest, request: Request) -> RunStartResponse:
    registry = request.app.state.registry
    validation = validate_graph(payload.graph, registry)
    if not validation.valid:
        raise HTTPException(status_code=400, detail={"code": "invalid_dag", "errors": [item.model_dump() for item in validation.errors]})

    session = request.app.state.sessions.create(payload.graph, payload.run_config)
    return RunStartResponse(session_id=session.session_id, websocket_url=f"/ws/run/{session.session_id}")


@router.post("/stop", response_model=RunStopResponse)
async def stop_run(payload: RunStopRequest, request: Request) -> RunStopResponse:
    stopped = request.app.state.sessions.stop(payload.session_id)
    if not stopped:
        raise HTTPException(status_code=404, detail={"code": "missing_session", "message": "Run session not found"})
    return RunStopResponse(stopped=True)
