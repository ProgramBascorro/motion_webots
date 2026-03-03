"use client";

import { useEffect, useMemo, useState, useTransition } from "react";
import type { Connection } from "@xyflow/react";
import type { PipelineGraph } from "@vision-lab/shared";

import { fetchNodeTypes, startRun, uploadFiles, validateGraph } from "@/lib/api";
import { autoLayout } from "@/lib/graph";
import { presetOptions } from "@/lib/presets";
import { GraphCanvas } from "@/components/graph/GraphCanvas";
import { InspectorPanel } from "@/components/inspector/InspectorPanel";
import { LogPanel } from "@/components/logs/LogPanel";
import { PreviewPanel } from "@/components/preview/PreviewPanel";
import { SidebarPalette } from "@/components/shell/SidebarPalette";
import { StatusBar } from "@/components/status/StatusBar";
import { Toolbar } from "@/components/toolbar/Toolbar";
import { useRunSession } from "@/features/run/useRunSession";
import { useEditorStore } from "@/store/editor-store";

export default function HomePage() {
  const [isPending, startTransition] = useTransition();
  const [selectedPreset, setSelectedPreset] = useState(presetOptions[0]?.id ?? "");
  const graph = useEditorStore((state) => state.graph);
  const nodes = useEditorStore((state) => state.nodes);
  const edges = useEditorStore((state) => state.edges);
  const nodeTypes = useEditorStore((state) => state.nodeTypes);
  const addNode = useEditorStore((state) => state.addNode);
  const setNodeTypes = useEditorStore((state) => state.setNodeTypes);
  const setGraph = useEditorStore((state) => state.setGraph);
  const updateValidationErrors = useEditorStore((state) => state.updateValidationErrors);
  const runState = useEditorStore((state) => state.runState);
  const setRunState = useEditorStore((state) => state.setRunState);
  const sessionId = useEditorStore((state) => state.sessionId);
  const addUploads = useEditorStore((state) => state.addUploads);
  const addLog = useEditorStore((state) => state.addLog);
  const onNodesChange = useEditorStore((state) => state.onNodesChange);
  const { connect, disconnect } = useRunSession();

  useEffect(() => {
    void fetchNodeTypes()
      .then((items) => setNodeTypes(items))
      .catch((error: Error) => addLog(error.message));
  }, [addLog, setNodeTypes]);

  useEffect(() => {
    if (nodeTypes.length > 0 && graph.nodes.length === 0) {
      setGraph(presetOptions[0].graph);
    }
  }, [graph.nodes.length, nodeTypes.length, setGraph]);

  const nodeTypeMap = useMemo(() => new Map(nodeTypes.map((item) => [item.type, item])), [nodeTypes]);

  const validateCurrentGraph = async () => {
    const result = await validateGraph(graph);
    updateValidationErrors(result.errors);
    if (!result.valid) {
      addLog(result.errors[0]?.message ?? "Graph validation failed");
    }
    return result.valid;
  };

  const handleRun = async () => {
    setRunState("starting");
    const valid = await validateCurrentGraph();
    if (!valid) {
      setRunState("error");
      return;
    }
    try {
      const response = await startRun(graph, { mode: "offline", preview_max_width: 480 });
      connect(response.session_id, response.websocket_url);
    } catch (error) {
      setRunState("error");
      addLog(error instanceof Error ? error.message : "Run start failed");
    }
  };

  const handleStop = async () => {
    setRunState("stopping");
    await disconnect(sessionId);
  };

  const handleSave = () => {
    const payload = JSON.stringify(graph, null, 2);
    const blob = new Blob([payload], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = `${graph.name.replace(/\s+/g, "_").toLowerCase() || "pipeline"}.json`;
    anchor.click();
    URL.revokeObjectURL(url);
  };

  const handleLoad = async (file: File) => {
    const payload = JSON.parse(await file.text()) as PipelineGraph;
    setGraph(payload);
    void validateCurrentGraph();
  };

  const handleUploadAssets = async (files: File[]) => {
    if (!files.length) {
      return;
    }
    try {
      const uploaded = await uploadFiles(files);
      addUploads(uploaded);
      uploaded.forEach((item) => addLog(`Uploaded ${item.filename} -> ${item.fileId}`));
    } catch (error) {
      addLog(error instanceof Error ? error.message : "Upload failed");
    }
  };

  const handlePresetChange = (presetId: string) => {
    setSelectedPreset(presetId);
    const preset = presetOptions.find((item) => item.id === presetId);
    if (preset) {
      setGraph(preset.graph);
      void validateCurrentGraph();
    }
  };

  const handleAutoLayout = () => {
    const laidOutNodes = autoLayout(nodes, edges);
    startTransition(() => {
      onNodesChange(
        laidOutNodes.map((node) => ({
          id: node.id,
          type: "position",
          position: node.position,
          dragging: false,
        })),
      );
    });
  };

  const isValidConnection = (connection: Connection) => {
    if (!connection.source || !connection.target || connection.source === connection.target) {
      return false;
    }
    const source = graph.nodes.find((node) => node.id === connection.source);
    const target = graph.nodes.find((node) => node.id === connection.target);
    if (!source || !target) {
      return false;
    }
    const sourceType = nodeTypeMap.get(source.type);
    const targetType = nodeTypeMap.get(target.type);
    if (!sourceType || !targetType) {
      return false;
    }
    const currentInputCount = graph.edges.filter((edge) => edge.target === target.id).length;
    return (
      currentInputCount < targetType.maxInputs &&
      sourceType.producedOutputKinds.some((kind) => targetType.acceptedInputKinds.includes(kind))
    );
  };

  return (
    <main className="min-h-screen p-4 text-white md:p-6">
      <div className="mx-auto flex max-w-[1800px] flex-col gap-4">
        <div className="flex items-center justify-between gap-3">
          <div>
            <div className="text-xs uppercase tracking-[0.25em] text-sky-300">Bascorro Studio Vision Lab</div>
            <h1 className="mt-2 text-3xl font-semibold">Visual CV pipeline debugger for robotics experiments</h1>
          </div>
          <div className="rounded-3xl border border-white/10 bg-slate-950/70 px-4 py-3 text-right text-sm text-slate-300">
            <div>Graph: {graph.name}</div>
            <div className="text-xs text-slate-500">{graph.nodes.length} nodes / {graph.edges.length} edges</div>
          </div>
        </div>

        <Toolbar
          onRun={handleRun}
          onStop={handleStop}
          onSave={handleSave}
          onLoad={handleLoad}
          onAutoLayout={handleAutoLayout}
          onUploadAssets={handleUploadAssets}
          runDisabled={runState === "running" || runState === "starting" || nodeTypes.length === 0}
          stopDisabled={!sessionId}
        >
          <div className="flex items-center gap-2">
            <label className="text-xs uppercase tracking-[0.2em] text-slate-400">Preset</label>
            <select
              value={selectedPreset}
              onChange={(event) => handlePresetChange(event.target.value)}
              className="rounded-2xl border border-white/10 bg-slate-950 px-3 py-2 text-sm"
            >
              {presetOptions.map((preset) => (
                <option key={preset.id} value={preset.id}>
                  {preset.label}
                </option>
              ))}
            </select>
          </div>
        </Toolbar>

        <div className="grid min-h-[720px] grid-cols-1 gap-4 xl:grid-cols-[260px_minmax(0,1fr)_340px]">
          <SidebarPalette nodeTypes={nodeTypes} onAddNode={addNode} />
          <section className="panel rounded-3xl p-3">
            <div className="mb-3 flex items-center justify-between px-2">
              <div>
                <div className="text-xs uppercase tracking-[0.18em] text-slate-400">Pipeline Canvas</div>
                <div className="text-sm text-slate-300">Build a DAG and inspect each node as frames move through it.</div>
              </div>
              {isPending ? <div className="text-xs text-slate-400">Laying out...</div> : null}
            </div>
            <div className="h-[640px] overflow-hidden rounded-3xl border border-white/10 bg-[#06101d]">
              <GraphCanvas isValidConnection={isValidConnection} />
            </div>
          </section>
          <div className="grid gap-4">
            <InspectorPanel />
            <PreviewPanel />
          </div>
        </div>

        <div className="grid gap-4 xl:grid-cols-[minmax(0,1fr)_440px]">
          <StatusBar />
          <LogPanel />
        </div>
      </div>
    </main>
  );
}
