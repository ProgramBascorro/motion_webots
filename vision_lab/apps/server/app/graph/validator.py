from __future__ import annotations

from collections import Counter
from pathlib import Path

from app.core.models import GraphValidationResponse, PipelineGraph, ValidationIssue
from app.graph.schema import ensure_graph_version
from app.graph.slots import incoming_edges
from app.graph.topo import topological_sort
from app.nodes.registry import NodeRegistry


INPUT_NODE_TYPES = {"image_input", "video_input", "webcam_input", "folder_replay_input"}


def validate_graph(graph: PipelineGraph, registry: NodeRegistry) -> GraphValidationResponse:
    errors: list[ValidationIssue] = []
    if not ensure_graph_version(graph):
        errors.append(ValidationIssue(code="invalid_version", message="Graph version must be 1.0"))
    if not graph.nodes:
        errors.append(ValidationIssue(code="missing_input_node", message="Pipeline requires at least one input node"))

    node_ids = [node.id for node in graph.nodes]
    edge_ids = [edge.id for edge in graph.edges]
    for node_id, count in Counter(node_ids).items():
        if count > 1:
            errors.append(ValidationIssue(code="duplicate_node_id", message=f"Duplicate node id: {node_id}", node_id=node_id))
    for edge_id, count in Counter(edge_ids).items():
        if count > 1:
            errors.append(ValidationIssue(code="duplicate_edge_id", message=f"Duplicate edge id: {edge_id}", edge_id=edge_id))

    node_map = {node.id: node for node in graph.nodes}
    incoming = incoming_edges(graph)

    for node in graph.nodes:
        definition = registry.get(node.type)
        if definition is None:
            errors.append(ValidationIssue(code="unknown_node_type", message=f"Unknown node type: {node.type}", node_id=node.id))
            continue
        if node.type in INPUT_NODE_TYPES and incoming.get(node.id):
            errors.append(ValidationIssue(code="input_has_incoming", message="Input nodes cannot have incoming edges", node_id=node.id))
        if node.type in {"image_input", "video_input"} and not str(node.params.get("file_id", "")).strip():
            errors.append(ValidationIssue(code="missing_upload", message="Input file_id is required", node_id=node.id))
        if node.type == "folder_replay_input" and not node.params.get("file_ids"):
            errors.append(ValidationIssue(code="missing_upload", message="Folder replay needs file_ids", node_id=node.id))
        if node.type in {"onnx_detection", "onnx_segmentation"}:
            use_mock = bool(node.params.get("use_mock", True))
            model_path = str(node.params.get("model_path", "")).strip()
            if not use_mock and not model_path:
                errors.append(ValidationIssue(code="missing_model_file", message="model_path is required when mock mode is disabled", node_id=node.id))
            if model_path and not use_mock and not Path(model_path).exists():
                errors.append(ValidationIssue(code="missing_model_file", message=f"Model file not found: {model_path}", node_id=node.id))

    if graph.nodes and not any(node.type in INPUT_NODE_TYPES for node in graph.nodes):
        errors.append(ValidationIssue(code="missing_input_node", message="Pipeline requires an input node"))

    for edge in graph.edges:
        if edge.source == edge.target:
            errors.append(ValidationIssue(code="self_loop", message="Self-loop edges are not allowed", edge_id=edge.id))
            continue
        source = node_map.get(edge.source)
        target = node_map.get(edge.target)
        if source is None or target is None:
            errors.append(ValidationIssue(code="missing_node", message="Edge references missing node", edge_id=edge.id))
            continue
        source_def = registry.get(source.type)
        target_def = registry.get(target.type)
        if source_def is None or target_def is None:
            continue
        current_input_count = len(incoming.get(target.id, []))
        if current_input_count > target_def.max_inputs:
            errors.append(ValidationIssue(code="too_many_inputs", message="Target node exceeds max inputs", edge_id=edge.id, node_id=target.id))
        compatible = any(kind in target_def.accepted_input_kinds for kind in source_def.produced_output_kinds)
        if not compatible:
            errors.append(
                ValidationIssue(
                    code="invalid_connection",
                    message=f"Cannot connect {source.type} to {target.type}",
                    edge_id=edge.id,
                    node_id=target.id,
                )
            )

    if errors:
        return GraphValidationResponse(valid=False, errors=errors, topo_order=None)

    try:
        order = topological_sort(graph)
    except ValueError:
        errors.append(ValidationIssue(code="invalid_dag", message="Graph must be acyclic"))
        return GraphValidationResponse(valid=False, errors=errors, topo_order=None)

    return GraphValidationResponse(valid=True, errors=[], topo_order=order)
