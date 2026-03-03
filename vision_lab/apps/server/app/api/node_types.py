from __future__ import annotations

from fastapi import APIRouter, Request

from app.core.models import NodeTypeResponse

router = APIRouter(prefix="/api", tags=["node-types"])


@router.get("/node-types", response_model=list[NodeTypeResponse])
async def list_node_types(request: Request) -> list[NodeTypeResponse]:
    registry = request.app.state.registry
    return [
        NodeTypeResponse(
            type=item.type,
            display_name=item.display_name,
            category=item.category,
            accepted_input_kinds=item.accepted_input_kinds,
            produced_output_kinds=item.produced_output_kinds,
            params_schema=item.params_schema,
            previewable=item.previewable,
            max_inputs=item.max_inputs,
        )
        for item in registry.list()
    ]
