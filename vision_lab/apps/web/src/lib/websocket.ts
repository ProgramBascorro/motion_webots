import { buildWebSocketUrl } from "@/lib/api";

export function connectRunSocket(path: string, onMessage: (payload: Record<string, unknown>) => void): WebSocket {
  const socket = new WebSocket(buildWebSocketUrl(path));
  socket.onmessage = (event) => {
    onMessage(JSON.parse(event.data) as Record<string, unknown>);
  };
  return socket;
}
