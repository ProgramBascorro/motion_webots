import type { PipelineGraph } from "./graph-schema";

export const emptyGraph = (): PipelineGraph => ({
  id: "graph-empty",
  name: "Untitled Pipeline",
  version: "1.0",
  nodes: [],
  edges: [],
});
