"use client";

import { useEditorStore } from "@/store/editor-store";

export function StatusBar() {
  const runState = useEditorStore((state) => state.runState);
  const metrics = useEditorStore((state) => state.metrics);
  const finalSummary = useEditorStore((state) => state.finalSummary);
  const statusMessage = useEditorStore((state) => state.statusMessage);
  const processedFrames = useEditorStore((state) => state.processedFrames);

  const metricEntries = Object.values(metrics).slice(0, 4);

  return (
    <section className="panel rounded-3xl px-5 py-4">
      <div className="flex flex-wrap items-center gap-3">
        <div className="rounded-full bg-sky-400/10 px-3 py-1 text-xs uppercase tracking-[0.16em] text-sky-300">
          {runState}
        </div>
        {statusMessage ? (
          <div className="rounded-full border border-sky-300/20 bg-sky-400/10 px-3 py-1 text-xs text-sky-100">
            {runState === "running" && processedFrames === 0 ? "Working: " : ""}
            {statusMessage}
          </div>
        ) : null}
        {metricEntries.map((entry) => (
          <div key={entry.nodeId} className="rounded-full border border-white/10 px-3 py-1 text-xs text-slate-300">
            {entry.nodeId}: {entry.latencyMs.toFixed(1)} ms
            {entry.fpsEstimate ? ` / ${entry.fpsEstimate.toFixed(1)} FPS` : ""}
          </div>
        ))}
        {finalSummary ? (
          <div className="rounded-full border border-emerald-300/30 bg-emerald-400/10 px-3 py-1 text-xs text-emerald-200">
            Avg latency: {Number(finalSummary.avg_latency_ms ?? 0).toFixed(1)} ms
          </div>
        ) : null}
      </div>
    </section>
  );
}
