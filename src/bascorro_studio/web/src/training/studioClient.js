import ROSLIB from "roslib";

// Permintaan ke apply_node (/bascorro_studio/request) yang menunggu jawabannya
// sendiri di /bascorro_studio/result lewat request_id. Balasan berawalan `prefix`
// tanpa penanya (mis. daftar percobaan sesudah ditambah dari tab lain) diteruskan
// ke onResult.

const REQUEST_TOPIC = "/bascorro_studio/request";
const RESULT_TOPIC = "/bascorro_studio/result";

export function createStudioClient(ros, { prefix, onResult, timeoutMs = 10000 }) {
  const request = new ROSLIB.Topic({ ros, name: REQUEST_TOPIC, messageType: "std_msgs/String" });
  const result = new ROSLIB.Topic({ ros, name: RESULT_TOPIC, messageType: "std_msgs/String" });
  const waiting = new Map();
  let counter = 0;

  result.subscribe((msg) => {
    let payload;
    try {
      payload = JSON.parse(msg.data);
    } catch {
      return;
    }
    if (typeof payload?.action !== "string" || !payload.action.startsWith(prefix)) return;
    onResult?.(payload);
    const entry = waiting.get(payload.request_id);
    if (!entry) return;
    waiting.delete(payload.request_id);
    window.clearTimeout(entry.timer);
    if (payload.ok === false) entry.reject(new Error(payload.message || `${payload.action} gagal`));
    else entry.resolve(payload);
  });

  const ask = (payload) => new Promise((resolve, reject) => {
    counter += 1;
    const id = `${prefix}${Date.now().toString(36)}-${counter}`;
    const timer = window.setTimeout(() => {
      waiting.delete(id);
      reject(new Error(`${payload.action}: server Studio (apply_node) tidak menjawab`));
    }, timeoutMs);
    waiting.set(id, { resolve, reject, timer });
    request.publish(new ROSLIB.Message({ data: JSON.stringify({ ...payload, request_id: id }) }));
  });

  const close = () => {
    result.unsubscribe();
    waiting.forEach((entry) => {
      window.clearTimeout(entry.timer);
      entry.reject(new Error("koneksi ROS berganti"));
    });
    waiting.clear();
  };

  return { ask, close };
}
