"use client";

import { create } from "zustand";
import type { Edge, Node } from "@xyflow/react";
import { addEdge, applyEdgeChanges, applyNodeChanges } from "@xyflow/react";
import { emptyGraph, type NodeTypeDefinition, type PipelineGraph, type ValidationIssue } from "@vision-lab/shared";

import type { MetricEntry, PreviewEntry, RunState, UploadEntry } from "@/types/ui";
import { flowToGraph, graphToFlow } from "@/lib/graph";

type EditorStore = {
  graph: PipelineGraph;
  nodes: Node[];
  edges: Edge[];
  selectedNodeId: string | null;
  nodeTypes: NodeTypeDefinition[];
  validationErrors: ValidationIssue[];
  previews: Record<string, PreviewEntry>;
  metrics: Record<string, MetricEntry>;
  uploads: Record<string, UploadEntry>;
  runState: RunState;
  sessionId: string | null;
  logs: string[];
  finalSummary: Record<string, unknown> | null;
  statusMessage: string | null;
  processedFrames: number;
  setNodeTypes: (nodeTypes: NodeTypeDefinition[]) => void;
  setGraph: (graph: PipelineGraph) => void;
  onNodesChange: (changes: Parameters<typeof applyNodeChanges>[0]) => void;
  onEdgesChange: (changes: Parameters<typeof applyEdgeChanges>[0]) => void;
  onConnect: (connection: Parameters<typeof addEdge>[0]) => void;
  selectNode: (nodeId: string | null) => void;
  updateNodeParams: (nodeId: string, params: Record<string, unknown>) => void;
  updateValidationErrors: (errors: ValidationIssue[]) => void;
  addNode: (definition: NodeTypeDefinition) => void;
  updateNodePositionGraph: () => void;
  setRunState: (state: RunState) => void;
  setSessionId: (sessionId: string | null) => void;
  upsertPreview: (entry: PreviewEntry) => void;
  upsertMetric: (entry: MetricEntry) => void;
  addUploads: (uploads: UploadEntry[]) => void;
  addLog: (message: string) => void;
  setFinalSummary: (summary: Record<string, unknown> | null) => void;
  setStatusMessage: (message: string | null) => void;
  setProcessedFrames: (count: number) => void;
};

function syncGraph(nodes: Node[], edges: Edge[], graph: PipelineGraph): PipelineGraph {
  return flowToGraph(nodes, edges, graph);
}

export const useEditorStore = create<EditorStore>((set) => ({
  graph: emptyGraph(),
  ...graphToFlow(emptyGraph()),
  selectedNodeId: null,
  nodeTypes: [],
  validationErrors: [],
  previews: {},
  metrics: {},
  uploads: {},
  runState: "idle",
  sessionId: null,
  logs: [],
  finalSummary: null,
  statusMessage: null,
  processedFrames: 0,
  setNodeTypes: (nodeTypes) =>
    set((state) => ({
      nodeTypes,
      nodes: state.nodes.map((node) => {
        const definition = nodeTypes.find((item) => item.type === node.data?.nodeType);
        return definition
          ? {
              ...node,
              data: {
                ...node.data,
                category: definition.category,
              },
            }
          : node;
      }),
    })),
  setGraph: (graph) =>
    set({
      graph,
      ...graphToFlow(graph),
      selectedNodeId: null,
      previews: {},
      metrics: {},
      finalSummary: null,
      statusMessage: null,
      processedFrames: 0,
    }),
  onNodesChange: (changes) =>
    set((state) => {
      const nodes = applyNodeChanges(changes, state.nodes);
      return { nodes, graph: syncGraph(nodes, state.edges, state.graph) };
    }),
  onEdgesChange: (changes) =>
    set((state) => {
      const edges = applyEdgeChanges(changes, state.edges);
      return { edges, graph: syncGraph(state.nodes, edges, state.graph) };
    }),
  onConnect: (connection) =>
    set((state) => {
      const edgeId = `edge_${crypto.randomUUID().slice(0, 8)}`;
      const edges = addEdge({ ...connection, id: edgeId }, state.edges);
      return { edges, graph: syncGraph(state.nodes, edges, state.graph) };
    }),
  selectNode: (selectedNodeId) => set({ selectedNodeId }),
  updateNodeParams: (nodeId, params) =>
    set((state) => {
      const graph = {
        ...state.graph,
        nodes: state.graph.nodes.map((node) => (node.id === nodeId ? { ...node, params } : node)),
      };
      return { graph };
    }),
  updateValidationErrors: (validationErrors) => set({ validationErrors }),
  addNode: (definition) =>
    set((state) => {
      const id = `${definition.type}_${state.nodes.length + 1}`;
      const node: Node = {
        id,
        type: "visionNode",
        position: { x: 120 + state.nodes.length * 20, y: 120 + state.nodes.length * 14 },
        data: { label: definition.displayName, nodeType: definition.type, category: definition.category },
      };
      const graph: PipelineGraph = {
        ...state.graph,
        nodes: [
          ...state.graph.nodes,
          {
            id,
            type: definition.type as PipelineGraph["nodes"][number]["type"],
            label: definition.displayName,
            position: node.position,
            params: Object.fromEntries(definition.paramsSchema.map((field) => [field.key, field.defaultValue ?? ""])),
          },
        ],
      };
      return { nodes: [...state.nodes, node], graph };
    }),
  updateNodePositionGraph: () =>
    set((state) => ({
      graph: syncGraph(state.nodes, state.edges, state.graph),
    })),
  setRunState: (runState) => set({ runState }),
  setSessionId: (sessionId) => set({ sessionId }),
  upsertPreview: (entry) =>
    set((state) => ({ previews: { ...state.previews, [entry.nodeId]: entry } })),
  upsertMetric: (entry) =>
    set((state) => ({ metrics: { ...state.metrics, [entry.nodeId]: entry } })),
  addUploads: (uploads) =>
    set((state) => ({
      uploads: {
        ...state.uploads,
        ...Object.fromEntries(uploads.map((upload) => [upload.fileId, upload])),
      },
    })),
  addLog: (message) => set((state) => ({ logs: [message, ...state.logs].slice(0, 80) })),
  setFinalSummary: (finalSummary) => set({ finalSummary }),
  setStatusMessage: (statusMessage) => set({ statusMessage }),
  setProcessedFrames: (processedFrames) => set({ processedFrames }),
}));
