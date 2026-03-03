"use client";

import { Background, Controls, MiniMap, ReactFlow, ReactFlowProvider } from "@xyflow/react";
import type { Connection } from "@xyflow/react";
import "@xyflow/react/dist/style.css";

import { useEditorStore } from "@/store/editor-store";
import { VisionNode } from "@/components/graph/VisionNode";

const nodeTypes = { visionNode: VisionNode };

export function GraphCanvas({
  isValidConnection,
}: {
  isValidConnection: (connection: Connection) => boolean;
}) {
  const nodes = useEditorStore((state) => state.nodes);
  const edges = useEditorStore((state) => state.edges);
  const onNodesChange = useEditorStore((state) => state.onNodesChange);
  const onEdgesChange = useEditorStore((state) => state.onEdgesChange);
  const onConnect = useEditorStore((state) => state.onConnect);
  const selectNode = useEditorStore((state) => state.selectNode);

  return (
    <ReactFlowProvider>
      <ReactFlow
        nodes={nodes}
        edges={edges}
        nodeTypes={nodeTypes}
        onNodesChange={onNodesChange}
        onEdgesChange={onEdgesChange}
        onConnect={onConnect}
        onNodeClick={(_, node) => selectNode(node.id)}
        onPaneClick={() => selectNode(null)}
        fitView
        isValidConnection={isValidConnection}
      >
        <MiniMap className="!bg-slate-950/90" />
        <Controls />
        <Background gap={24} size={1.4} color="rgba(147,197,253,0.18)" />
      </ReactFlow>
    </ReactFlowProvider>
  );
}
