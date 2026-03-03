"use client";

import { useRef } from "react";

import type { WsMessage } from "@vision-lab/shared";

import { connectRunSocket } from "@/lib/websocket";
import { stopRun } from "@/lib/api";
import { useEditorStore } from "@/store/editor-store";

export function useRunSession() {
  const socketRef = useRef<WebSocket | null>(null);
  const upsertPreview = useEditorStore((state) => state.upsertPreview);
  const upsertMetric = useEditorStore((state) => state.upsertMetric);
  const addLog = useEditorStore((state) => state.addLog);
  const setRunState = useEditorStore((state) => state.setRunState);
  const setFinalSummary = useEditorStore((state) => state.setFinalSummary);
  const setSessionId = useEditorStore((state) => state.setSessionId);
  const setStatusMessage = useEditorStore((state) => state.setStatusMessage);
  const setProcessedFrames = useEditorStore((state) => state.setProcessedFrames);

  const connect = (sessionId: string, websocketPath: string) => {
    socketRef.current?.close();
    setProcessedFrames(0);
    setFinalSummary(null);
    setStatusMessage("Connecting to runner");
    const socket = connectRunSocket(websocketPath, (payload) => {
      const message = payload as WsMessage;
      switch (message.type) {
        case "run_started":
          setRunState("running");
          setStatusMessage("Run started, preparing pipeline");
          addLog("Run started");
          break;
        case "run_progress":
          setProcessedFrames(message.frame_index);
          setStatusMessage(`Processing frames (${message.frame_index} completed)`);
          addLog(`Frame ${message.frame_index} processed`);
          break;
        case "node_preview":
          upsertPreview({
            nodeId: message.node_id,
            frameId: message.frame_id,
            previewType: message.preview_type,
            previewPayload: (message.preview_payload as Record<string, unknown>) ?? null,
          });
          break;
        case "node_metrics":
          upsertMetric({
            nodeId: message.node_id,
            latencyMs: message.latency_ms,
            fpsEstimate: message.fps_estimate,
            outputKind: message.output_kind,
          });
          break;
        case "node_error":
          setRunState("error");
          setStatusMessage(message.message);
          addLog(message.message);
          break;
        case "run_finished":
          setRunState("finished");
          setFinalSummary(message.summary);
          setStatusMessage(`Finished after ${message.total_frames} frame(s)`);
          addLog(`Run finished. ${message.total_frames} frames processed.`);
          break;
        case "run_stopped":
          setRunState("idle");
          setStatusMessage(message.reason);
          addLog(message.reason);
          break;
        case "run_log":
          setStatusMessage(message.message);
          addLog(message.message);
          break;
      }
    });
    socketRef.current = socket;
    setSessionId(sessionId);
  };

  const disconnect = async (sessionId: string | null) => {
    if (sessionId) {
      await stopRun(sessionId);
    }
    socketRef.current?.close();
    socketRef.current = null;
    setSessionId(null);
    setRunState("idle");
    setStatusMessage(null);
    setProcessedFrames(0);
  };

  return { connect, disconnect };
}
