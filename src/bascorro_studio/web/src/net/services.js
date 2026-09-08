export function callRosService(service, args = {}, { signal, timeoutMs = 4000 } = {}) {
  return new Promise((resolve, reject) => {
    const ros = service?.ros;
    if (!ros?.isConnected || signal?.aborted) {
      reject(new Error("Koneksi ROS tidak aktif atau operasi dibatalkan"));
      return;
    }
    const requestId = `bascorro_service:${service.name}:${++ros.idCounter}`;
    let timer;
    const finish = (error, result) => {
      clearTimeout(timer);
      ros.off(requestId, receive);
      ros.off("close", disconnected);
      ros.off("error", disconnected);
      signal?.removeEventListener("abort", cancelled);
      if (error) reject(error);
      else resolve(result);
    };
    const receive = (message) => {
      if (message.result === false) {
        const detail = typeof message.values === "string" ? message.values : JSON.stringify(message.values);
        finish(new Error(`${service.name}: ${detail || "service gagal"}`));
      } else {
        finish(null, message.values);
      }
    };
    const disconnected = () => finish(new Error("Koneksi ROS terputus; hasil operasi belum terkonfirmasi"));
    const cancelled = () => finish(new Error("Operasi dibatalkan"));
    ros.once(requestId, receive);
    ros.once("close", disconnected);
    ros.once("error", disconnected);
    signal?.addEventListener("abort", cancelled, { once: true });
    timer = setTimeout(() => finish(new Error(`${service.name}: tidak menjawab dalam ${timeoutMs / 1000} detik; hasil belum terkonfirmasi`)), timeoutMs);
    try {
      ros.callOnConnection({ op: "call_service", id: requestId, service: service.name, type: service.serviceType, args });
    } catch (error) {
      finish(error);
    }
  });
}

export function waitForRobot(delayMs, signal) {
  return new Promise((resolve, reject) => {
    if (signal?.aborted) {
      reject(new Error("Operasi dibatalkan"));
      return;
    }
    const cancelled = () => {
      clearTimeout(timer);
      reject(new Error("Operasi dibatalkan"));
    };
    const timer = setTimeout(() => {
      signal?.removeEventListener("abort", cancelled);
      resolve();
    }, delayMs);
    signal?.addEventListener("abort", cancelled, { once: true });
  });
}
