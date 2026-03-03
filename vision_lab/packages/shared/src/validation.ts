import type { OutputKind, PipelineGraph } from "./graph-schema";
import type { NodeTypeDefinition } from "./node-types";

export function getNodeMap(graph: PipelineGraph): Map<string, PipelineGraph["nodes"][number]> {
  return new Map(graph.nodes.map((node) => [node.id, node]));
}

export function canConnectKinds(
  sourceKinds: OutputKind[],
  targetDefinition?: NodeTypeDefinition,
  currentInputCount = 0,
): boolean {
  if (!targetDefinition) {
    return false;
  }
  if (currentInputCount >= targetDefinition.maxInputs) {
    return false;
  }
  return sourceKinds.some((kind) => targetDefinition.acceptedInputKinds.includes(kind));
}
