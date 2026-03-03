"use client";

import { useMemo } from "react";

import { useEditorStore } from "@/store/editor-store";

export function PreviewPanel() {
  const selectedNodeId = useEditorStore((state) => state.selectedNodeId);
  const previews = useEditorStore((state) => state.previews);
  const graph = useEditorStore((state) => state.graph);

  const selectedPreview = selectedNodeId ? previews[selectedNodeId] : undefined;
  const selectedNode = useMemo(() => graph.nodes.find((node) => node.id === selectedNodeId), [graph.nodes, selectedNodeId]);

  return (
    <section className="panel rounded-3xl p-5">
      <div className="flex items-center justify-between">
        <div>
          <div className="text-xs uppercase tracking-[0.2em] text-sky-300">Preview</div>
          <h3 className="mt-2 text-lg font-semibold">{selectedNode?.label ?? "No node selected"}</h3>
        </div>
        {selectedPreview?.frameId ? <div className="text-xs text-slate-400">{selectedPreview.frameId}</div> : null}
      </div>
      <div className="mt-4 min-h-[220px] rounded-3xl border border-white/10 bg-slate-950/70 p-3">
        {!selectedPreview ? (
          <div className="flex h-full items-center justify-center text-sm text-slate-400">Run the pipeline and select a node to inspect its latest output.</div>
        ) : selectedPreview.previewPayload?.kind === "image" ? (
          // eslint-disable-next-line @next/next/no-img-element
          <img
            src={String(selectedPreview.previewPayload.imageUrl)}
            alt="Node preview"
            className="max-h-[360px] w-full rounded-2xl object-contain"
          />
        ) : selectedPreview.previewPayload?.kind === "json" ? (
          <pre className="max-h-[360px] overflow-auto rounded-2xl bg-slate-950 p-3 text-xs text-slate-200">
            {JSON.stringify(selectedPreview.previewPayload.data, null, 2)}
          </pre>
        ) : (
          <div className="text-sm text-slate-300">{String(selectedPreview.previewPayload?.text ?? "")}</div>
        )}
      </div>
    </section>
  );
}
