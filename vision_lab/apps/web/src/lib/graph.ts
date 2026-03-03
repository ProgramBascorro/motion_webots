import dagre from "dagre";
import type { Edge, Node } from "@xyflow/react";
import type { GraphNode, PipelineGraph } from "@vision-lab/shared";

export function graphToFlow(graph: PipelineGraph): { nodes: Node[]; edges: Edge[] } {
  return {
    nodes: graph.nodes.map((node) => ({
      id: node.id,
      type: "visionNode",
      position: node.position,
      data: { label: node.label, nodeType: node.type },
    })),
    edges: graph.edges.map((edge) => ({ id: edge.id, source: edge.source, target: edge.target })),
  };
}

export function flowToGraph(nodes: Node[], edges: Edge[], previous: PipelineGraph): PipelineGraph {
  const nextNodes: GraphNode[] = nodes.map((node) => {
    const prior = previous.nodes.find((item) => item.id === node.id);
    return {
      id: node.id,
      type: String(node.data?.nodeType ?? prior?.type ?? "resize") as GraphNode["type"],
      label: String(node.data?.label ?? prior?.label ?? node.id),
      position: { x: node.position.x, y: node.position.y },
      params: prior?.params ?? {},
    };
  });

  return {
    ...previous,
    nodes: nextNodes,
    edges: edges.map((edge) => ({ id: edge.id, source: edge.source, target: edge.target })),
  };
}

export function autoLayout(nodes: Node[], edges: Edge[]): Node[] {
  const graph = new dagre.graphlib.Graph();
  graph.setDefaultEdgeLabel(() => ({}));
  graph.setGraph({ rankdir: "LR", ranksep: 100, nodesep: 45 });
  nodes.forEach((node) => graph.setNode(node.id, { width: 200, height: 76 }));
  edges.forEach((edge) => graph.setEdge(edge.source, edge.target));
  dagre.layout(graph);
  return nodes.map((node) => {
    const position = graph.node(node.id);
    return {
      ...node,
      position: {
        x: position.x - 100,
        y: position.y - 38,
      },
    };
  });
}
