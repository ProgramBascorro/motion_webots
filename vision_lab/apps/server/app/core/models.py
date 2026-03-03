from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


OutputKind = Literal["frame", "image", "detections", "mask", "json", "text", "metrics"]


class Position(BaseModel):
    x: float
    y: float


class GraphNode(BaseModel):
    id: str
    type: str
    label: str
    position: Position
    params: dict[str, Any] = Field(default_factory=dict)


class GraphEdge(BaseModel):
    id: str
    source: str
    target: str


class PipelineGraph(BaseModel):
    id: str
    name: str
    version: str = "1.0"
    nodes: list[GraphNode]
    edges: list[GraphEdge]


class ValidationIssue(BaseModel):
    code: str
    message: str
    node_id: str | None = None
    edge_id: str | None = None


class GraphValidationResponse(BaseModel):
    valid: bool
    errors: list[ValidationIssue] = Field(default_factory=list)
    topo_order: list[str] | None = None


class UploadedAssetRef(BaseModel):
    file_id: str
    filename: str
    media_type: str
    path: str
    size_bytes: int


class UploadResponse(BaseModel):
    files: list[UploadedAssetRef]


class RunConfig(BaseModel):
    mode: Literal["offline", "live"] = "offline"
    max_frames: int | None = None
    preview_max_width: int = 480


class RunStartRequest(BaseModel):
    graph: PipelineGraph
    run_config: RunConfig = Field(default_factory=RunConfig)


class RunStartResponse(BaseModel):
    session_id: str
    websocket_url: str
    accepted: bool = True


class RunStopRequest(BaseModel):
    session_id: str


class RunStopResponse(BaseModel):
    stopped: bool


class NodeMetricsModel(BaseModel):
    latency_ms: float
    fps_estimate: float | None = None


class NodeResultModel(BaseModel):
    node_id: str
    type: str
    success: bool
    output_kind: Literal["image", "detections", "mask", "json", "text", "metrics"]
    payload: Any = None
    preview: dict[str, Any] | None = None
    metrics: NodeMetricsModel
    error: str | None = None


class NodeTypeResponse(BaseModel):
    type: str
    display_name: str
    category: str
    accepted_input_kinds: list[str]
    produced_output_kinds: list[str]
    params_schema: list[dict[str, Any]]
    previewable: bool
    max_inputs: int
