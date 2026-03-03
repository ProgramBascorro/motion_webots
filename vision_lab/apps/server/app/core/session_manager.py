from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any

from fastapi import WebSocket

from app.core.models import PipelineGraph, RunConfig
from app.utils.ids import make_id


@dataclass
class SessionState:
    session_id: str
    graph: PipelineGraph
    run_config: RunConfig
    websocket: WebSocket | None = None
    task: asyncio.Task[Any] | None = None
    started: bool = False
    stop_requested: bool = False
    finished: bool = False
    summary: dict[str, Any] = field(default_factory=dict)


class SessionManager:
    def __init__(self) -> None:
        self._sessions: dict[str, SessionState] = {}

    def create(self, graph: PipelineGraph, run_config: RunConfig) -> SessionState:
        session = SessionState(session_id=make_id("run"), graph=graph, run_config=run_config)
        self._sessions[session.session_id] = session
        return session

    def get(self, session_id: str) -> SessionState | None:
        return self._sessions.get(session_id)

    def require(self, session_id: str) -> SessionState:
        session = self.get(session_id)
        if session is None:
            raise KeyError(session_id)
        return session

    def stop(self, session_id: str) -> bool:
        session = self.get(session_id)
        if session is None:
            return False
        session.stop_requested = True
        return True

    async def send(self, session_id: str, payload: dict[str, Any]) -> None:
        session = self.require(session_id)
        if session.websocket is not None:
            await session.websocket.send_json(payload)
