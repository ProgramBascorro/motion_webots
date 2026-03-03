from __future__ import annotations

import asyncio

from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect

router = APIRouter(tags=["websocket"])


@router.websocket("/ws/run/{session_id}")
async def run_websocket(websocket: WebSocket, session_id: str) -> None:
    await websocket.accept()
    app = websocket.app
    session = app.state.sessions.get(session_id)
    if session is None:
        await websocket.send_json({"type": "node_error", "session_id": session_id, "error_code": "missing_session", "message": "Run session not found"})
        await websocket.close(code=1008)
        return

    session.websocket = websocket
    if session.task is None:
        executor = app.state.executor
        session.task = asyncio.create_task(executor.execute(session))

    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        session.websocket = None
