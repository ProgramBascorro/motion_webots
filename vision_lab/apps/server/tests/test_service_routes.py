from app.core.models import GraphEdge, GraphNode, PipelineGraph, Position, RunConfig
from app.core.session_manager import SessionManager
from app.graph.validator import validate_graph
from app.nodes.registry import NodeRegistry


def test_valid_graph_passes_service_validation() -> None:
    graph = PipelineGraph(
        id="g1",
        name="Graph",
        version="1.0",
        nodes=[
            GraphNode(id="cam", type="webcam_input", label="Cam", position=Position(x=0, y=0), params={"device_index": 0}),
            GraphNode(id="resize", type="resize", label="Resize", position=Position(x=1, y=0), params={"width": 320, "height": 240}),
        ],
        edges=[GraphEdge(id="e1", source="cam", target="resize")],
    )

    result = validate_graph(graph, NodeRegistry())
    assert result.valid is True
    assert result.topo_order == ["cam", "resize"]


def test_session_manager_creates_session_for_valid_run() -> None:
    graph = PipelineGraph(
        id="g2",
        name="Graph",
        version="1.0",
        nodes=[GraphNode(id="cam", type="webcam_input", label="Cam", position=Position(x=0, y=0), params={"device_index": 0})],
        edges=[],
    )
    session = SessionManager().create(graph, RunConfig(mode="live"))
    assert session.session_id.startswith("run_")
    assert session.graph.id == "g2"
