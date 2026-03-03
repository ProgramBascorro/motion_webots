from __future__ import annotations

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.graphs import router as graphs_router
from app.api.node_types import router as node_types_router
from app.api.run import router as run_router
from app.api.uploads import router as uploads_router
from app.api.websocket import router as websocket_router
from app.lifespan import lifespan

app = FastAPI(title="Bascorro Studio Vision Lab", version="0.1.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(graphs_router)
app.include_router(node_types_router)
app.include_router(run_router)
app.include_router(uploads_router)
app.include_router(websocket_router)


@app.get("/api/health")
async def health() -> dict[str, str]:
    return {"status": "ok"}
