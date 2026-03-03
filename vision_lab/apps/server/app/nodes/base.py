from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


ExecutorFn = Callable[..., Any]


@dataclass
class NodeTypeDefinition:
    type: str
    display_name: str
    category: str
    accepted_input_kinds: list[str]
    produced_output_kinds: list[str]
    params_schema: list[dict[str, Any]]
    previewable: bool
    max_inputs: int
    executor: ExecutorFn | None = None
