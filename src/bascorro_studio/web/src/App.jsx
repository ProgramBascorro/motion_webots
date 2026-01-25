import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";
import {
  Activity,
  AlertCircle,
  Battery,
  Camera,
  Cpu,
  Gauge,
  HardDrive,
  History,
  LayoutDashboard,
  Menu,
  Play,
  Power,
  RefreshCw,
  Settings,
  StopCircle,
  Terminal,
  Video,
  Wifi,
  Zap,
} from "lucide-react";
import ActionEditor from "./ActionEditor.jsx";
import GamepadVisualizer from "./GamepadVisualizer.jsx";
import ChartPage from "./ChartPage.jsx";

const DEFAULT_ROSBRIDGE =
  import.meta.env.VITE_ROSBRIDGE_URL || "ws://localhost:9090";
const DEFAULT_OVERLAY_TOPIC =
  import.meta.env.VITE_OVERLAY_TOPIC || "/vision/yolo/debug";

const PARAM_TYPES = {
  bool: 1,
  int: 2,
  double: 3,
  string: 4,
};

const YOLO_PARAM_KEYS = [
  "ball_confidence_threshold",
  "goalpost_confidence_threshold",
  "robot_confidence_threshold",
];

// --- Utilities ---

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function formatNumber(value, digits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-";
  }
  return Number(value).toFixed(digits);
}

function formatPercent(value) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-%";
  }
  return `${Math.round(value)}%`;
}

function formatAge(seconds) {
  if (seconds === null || seconds === undefined || Number.isNaN(seconds)) {
    return "-";
  }
  if (seconds < 1) {
    return `${Math.round(seconds * 1000)}ms`;
  }
  return `${seconds.toFixed(1)}s`;
}

function decodeImage(msg) {
  if (!msg || !msg.data || !msg.width || !msg.height) {
    return null;
  }
  const { width, height, encoding } = msg;
  const bin = atob(msg.data);
  const raw = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i += 1) {
    raw[i] = bin.charCodeAt(i);
  }

  const pixels = new Uint8ClampedArray(width * height * 4);
  if (encoding === "rgb8" || encoding === "bgr8") {
    const isBgr = encoding === "bgr8";
    for (let i = 0; i < width * height; i += 1) {
      const offset = i * 3;
      const r = isBgr ? raw[offset + 2] : raw[offset];
      const g = raw[offset + 1];
      const b = isBgr ? raw[offset] : raw[offset + 2];
      const out = i * 4;
      pixels[out] = r;
      pixels[out + 1] = g;
      pixels[out + 2] = b;
      pixels[out + 3] = 255;
    }
    return new ImageData(pixels, width, height);
  }
  if (encoding === "bgra8") {
    for (let i = 0; i < width * height; i += 1) {
      const offset = i * 4;
      const out = i * 4;
      pixels[out] = raw[offset + 2];     // R
      pixels[out + 1] = raw[offset + 1]; // G
      pixels[out + 2] = raw[offset];     // B
      pixels[out + 3] = raw[offset + 3]; // A
    }
    return new ImageData(pixels, width, height);
  }
  if (encoding === "mono8") {
    for (let i = 0; i < width * height; i += 1) {
      const val = raw[i];
      const out = i * 4;
      pixels[out] = val;
      pixels[out + 1] = val;
      pixels[out + 2] = val;
      pixels[out + 3] = 255;
    }
    return new ImageData(pixels, width, height);
  }
  return null;
}

function makeParamValue(type, rawValue) {
  const value = String(rawValue ?? "").trim();
  if (type === "bool") {
    return { type: PARAM_TYPES.bool, bool_value: value === "true" };
  }
  if (type === "int") {
    return { type: PARAM_TYPES.int, integer_value: Number(value) };
  }
  if (type === "string") {
    return { type: PARAM_TYPES.string, string_value: value };
  }
  return { type: PARAM_TYPES.double, double_value: Number(value) };
}

function parseHealthMessage(message) {
  if (!message) return null;
  try {
    return JSON.parse(message);
  } catch (err) {
    console.error("Failed to parse health response", err);
    return { raw: message };
  }
}

// --- Components ---

const StatCard = ({ title, icon: Icon, value, subValue, status = "neutral" }) => {
  const statusStyles = {
    neutral: "border-gray-200 bg-white",
    danger: "border-red-200 bg-red-50",
    warning: "border-yellow-200 bg-yellow-50",
    success: "border-green-200 bg-green-50",
  };

  return (
    <div className={`p-5 rounded-2xl border shadow-sm transition-all hover:shadow-md ${statusStyles[status]}`}>
      <div className="flex justify-between items-center mb-2 text-gray-500">
        <span className="text-sm font-medium">{title}</span>
        {Icon && <Icon size={18} />}
      </div>
      <div>
        <span className="block text-2xl font-bold font-display text-gray-900">{value}</span>
        {subValue && <span className="text-xs text-gray-500 font-mono">{subValue}</span>}
      </div>
    </div>
  );
};

const SectionHeader = ({ title, children }) => (
  <div className="flex justify-between items-center mb-4">
    <h2 className="text-lg font-bold font-display text-gray-800">{title}</h2>
    <div className="flex gap-2">{children}</div>
  </div>
);

export default function App() {
  const [activeTab, setActiveTab] = useState("dashboard");
  const [rosUrl, setRosUrl] = useState(DEFAULT_ROSBRIDGE);
  const [rosState, setRosState] = useState("disconnected");
  
  // Data State
  const [metrics, setMetrics] = useState(null);
  const [events, setEvents] = useState([]);
  const [studioStatus, setStudioStatus] = useState("");
  const [studioError, setStudioError] = useState(false);
  const [healthResult, setHealthResult] = useState(null);
  const [healthError, setHealthError] = useState("");
  const [healthRunning, setHealthRunning] = useState(false);
  const [healthCheckedAt, setHealthCheckedAt] = useState(null);
  const [demoMode, setDemoMode] = useState("");
  const [demoCommand, setDemoCommand] = useState("");
  const [demoCommandAt, setDemoCommandAt] = useState(null);
  
  // Vision
  const [overlayTopic, setOverlayTopic] = useState(DEFAULT_OVERLAY_TOPIC);
  const [showOverlay, setShowOverlay] = useState(true);
  const [overlayStats, setOverlayStats] = useState({ fps: 0, lastFrameMs: null, dropped: 0 });
  
  // Tuning
  const [yoloParams, setYoloParams] = useState({
    ball_confidence_threshold: 0.2,
    goalpost_confidence_threshold: 0.5,
    robot_confidence_threshold: 0.2,
  });
  const [walkingParams, setWalkingParams] = useState({
    x_move_amplitude: 0.0,
    y_move_amplitude: 0.0,
    angle_move_amplitude: 0.0,
  });
  const [walkingFull, setWalkingFull] = useState(null);
  const [paramNode, setParamNode] = useState("op3_yolo_vision");
  const [paramName, setParamName] = useState("ball_confidence_threshold");
  const [paramType, setParamType] = useState("double");
  const [paramValue, setParamValue] = useState("0.2");
  
  // Input
  const [joyState, setJoyState] = useState(null);

  // Refs
  const rosRef = useRef(null);
  const overlayCanvasRef = useRef(null);
  const overlayLastRef = useRef({ stampMs: null, fps: 0, dropped: 0 });
  const showOverlayRef = useRef(true);
  
  // Topic/Service Refs
  const metricsSubRef = useRef(null);
  const eventsSubRef = useRef(null);
  const overlaySubRef = useRef(null);
  const joySubRef = useRef(null);
  const joyPubRef = useRef(null);
  const initPosePubRef = useRef(null);
  const walkingCommandPubRef = useRef(null);
  const torquePubRef = useRef(null);
  const walkingParamPubRef = useRef(null);
  const walkingGetServiceRef = useRef(null);
  const yoloSetServiceRef = useRef(null);
  const yoloGetServiceRef = useRef(null);
  const snapshotServiceRef = useRef(null);
  const bagStartServiceRef = useRef(null);
  const bagStopServiceRef = useRef(null);
  const healthCheckServiceRef = useRef(null);
  const demoModePubRef = useRef(null);
  const demoCommandPubRef = useRef(null);

  // --- Effects ---

  useEffect(() => {
    showOverlayRef.current = showOverlay;
  }, [showOverlay]);

  useEffect(() => {
    if (!rosUrl) return;

    const ros = new ROSLIB.Ros({ url: rosUrl });
    rosRef.current = ros;

    ros.on("connection", () => setRosState("connected"));
    ros.on("error", () => setRosState("error"));
    ros.on("close", () => setRosState("disconnected"));

    // Subscriptions
    metricsSubRef.current = new ROSLIB.Topic({
      ros,
      name: "/bascorro_studio/metrics",
      messageType: "std_msgs/String",
    });
    metricsSubRef.current.subscribe((msg) => {
      try {
        setMetrics(JSON.parse(msg.data));
      } catch (e) { console.error(e); }
    });

    eventsSubRef.current = new ROSLIB.Topic({
      ros,
      name: "/bascorro_studio/events",
      messageType: "std_msgs/String",
    });
    eventsSubRef.current.subscribe((msg) => {
      try {
        const payload = JSON.parse(msg.data);
        if (Array.isArray(payload.events)) setEvents(payload.events);
      } catch (e) { console.error(e); }
    });

    joySubRef.current = new ROSLIB.Topic({
      ros,
      name: "/joy",
      messageType: "sensor_msgs/Joy",
    });
    joySubRef.current.subscribe(setJoyState);

    // Publishers & Services
    joyPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/joy",
      messageType: "sensor_msgs/Joy",
    });
    initPosePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/base/ini_pose",
      messageType: "std_msgs/String",
    });
    walkingCommandPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/walking/command",
      messageType: "std_msgs/String",
    });
    torquePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/sync_write_item",
      messageType: "robotis_controller_msgs/SyncWriteItem",
    });
    walkingParamPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/walking/set_params",
      messageType: "op3_walking_module_msgs/WalkingParam",
    });
    walkingGetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/robotis/walking/get_params",
      serviceType: "op3_walking_module_msgs/srv/GetWalkingParam",
    });
    yoloSetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/op3_yolo_vision/set_parameters",
      serviceType: "rcl_interfaces/srv/SetParameters",
    });
    yoloGetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/op3_yolo_vision/get_parameters",
      serviceType: "rcl_interfaces/srv/GetParameters",
    });
    snapshotServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/bascorro_studio/snapshot",
      serviceType: "std_srvs/srv/Trigger",
    });
    bagStartServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/bascorro_studio/bag_start",
      serviceType: "std_srvs/srv/Trigger",
    });
    bagStopServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/bascorro_studio/bag_stop",
      serviceType: "std_srvs/srv/Trigger",
    });
    healthCheckServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/robotis/health_check",
      serviceType: "std_srvs/srv/Trigger",
    });
    demoModePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/mode_command",
      messageType: "std_msgs/String",
    });
    demoCommandPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/demo_command",
      messageType: "std_msgs/String",
    });

    // Load initial params
    if (yoloGetServiceRef.current) {
        const request = new ROSLIB.ServiceRequest({ names: YOLO_PARAM_KEYS });
        yoloGetServiceRef.current.callService(request, (result) => {
            if (result && Array.isArray(result.values)) {
                const next = { ...yoloParams };
                result.values.forEach((val, i) => {
                    if (val && typeof val.double_value === 'number') {
                        next[YOLO_PARAM_KEYS[i]] = val.double_value;
                    }
                });
                setYoloParams(next);
            }
        });
    }

    return () => {
      metricsSubRef.current?.unsubscribe();
      eventsSubRef.current?.unsubscribe();
      joySubRef.current?.unsubscribe();
      ros.close();
    };
  }, [rosUrl]);

  useEffect(() => {
    if (!rosRef.current || rosState === "disconnected") return;
    
    if (overlaySubRef.current) overlaySubRef.current.unsubscribe();
    
    overlaySubRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: overlayTopic,
      messageType: "sensor_msgs/Image",
    });

    overlaySubRef.current.subscribe((msg) => {
      if (!showOverlayRef.current) return;
      const imageData = decodeImage(msg);
      if (!imageData || !overlayCanvasRef.current) return;
      
      const canvas = overlayCanvasRef.current;
      if (canvas.width !== imageData.width || canvas.height !== imageData.height) {
        canvas.width = imageData.width;
        canvas.height = imageData.height;
      }
      const ctx = canvas.getContext("2d");
      ctx.putImageData(imageData, 0, 0);

      const now = performance.now();
      const last = overlayLastRef.current.stampMs;
      const fps = last ? 1000 / (now - last) : 0;
      overlayLastRef.current = { stampMs: now, fps, dropped: 0 };
      setOverlayStats(prev => ({ ...prev, fps, lastFrameMs: now }));
    });

    return () => overlaySubRef.current?.unsubscribe();
  }, [overlayTopic, rosState]);

  // --- Handlers ---

  const sendStatus = (msg, isError = false) => {
    setStudioStatus(msg);
    setStudioError(isError);
    setTimeout(() => { if(studioStatus === msg) setStudioStatus(""); }, 3000);
  };

  const handleInitPose = () => {
    if (initPosePubRef.current) {
      initPosePubRef.current.publish(new ROSLIB.Message({ data: "ini_pose" }));
      sendStatus("Init Pose Sent");
    }
  };

  const handleSoftStop = () => {
    if (walkingCommandPubRef.current) {
      walkingCommandPubRef.current.publish(new ROSLIB.Message({ data: "stop" }));
      sendStatus("Soft Stop Sent");
    }
  };

  const handleTorque = (enable) => {
    if (torquePubRef.current && metrics?.joint_names?.length) {
      const values = metrics.joint_names.map(() => (enable ? 1 : 0));
      torquePubRef.current.publish(new ROSLIB.Message({
        item_name: "torque_enable",
        joint_name: metrics.joint_names,
        value: values
      }));
      sendStatus(enable ? "Torque ON" : "Torque OFF");
    }
  };

  const pushDemoStatus = (label, mode = "") => {
    setDemoCommand(label);
    setDemoCommandAt(Date.now());
    if (mode) setDemoMode(mode);
  };

  const handleDemoMode = (mode) => {
    if (!demoModePubRef.current || rosState !== "connected") {
      sendStatus("Demo mode unavailable", true);
      return;
    }
    demoModePubRef.current.publish(new ROSLIB.Message({ data: mode }));
    pushDemoStatus(`mode ${mode}`, mode);
    sendStatus(`Demo mode: ${mode}`);
  };

  const handleDemoCommand = (command) => {
    if (!demoCommandPubRef.current || rosState !== "connected") {
      sendStatus("Demo command unavailable", true);
      return;
    }
    demoCommandPubRef.current.publish(new ROSLIB.Message({ data: command }));
    pushDemoStatus(`command ${command}`);
    sendStatus(`Demo ${command}`);
  };

  const handleHealthCheck = () => {
    if (!healthCheckServiceRef.current) {
      sendStatus("Health check unavailable", true);
      return;
    }
    setHealthRunning(true);
    setHealthError("");
    healthCheckServiceRef.current.callService(
      new ROSLIB.ServiceRequest({}),
      (res) => {
        setHealthRunning(false);
        if (res?.success) {
          setHealthResult(parseHealthMessage(res.message));
          setHealthCheckedAt(Date.now());
          sendStatus("Health check complete");
        } else {
          const message = res?.message || "Health check failed";
          setHealthError(message);
          sendStatus(message, true);
        }
      },
      (err) => {
        setHealthRunning(false);
        const message = err?.message || "Health check failed";
        setHealthError(message);
        sendStatus(message, true);
      }
    );
  };

  const publishJoy = useCallback((payload) => {
    if (joyPubRef.current && rosState === "connected") {
      const now = Date.now();
      joyPubRef.current.publish(new ROSLIB.Message({
        header: {
          stamp: { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1000000 },
          frame_id: "bascorro_studio"
        },
        axes: payload?.axes || [],
        buttons: payload?.buttons || []
      }));
      return true;
    }
    return false;
  }, [rosState]);

  const loadWalkingParams = () => {
    if (walkingGetServiceRef.current) {
      walkingGetServiceRef.current.callService(new ROSLIB.ServiceRequest({ get_param: true }), (res) => {
        if (res?.parameters) {
          setWalkingFull(res.parameters);
          setWalkingParams({
            x_move_amplitude: res.parameters.x_move_amplitude ?? 0,
            y_move_amplitude: res.parameters.y_move_amplitude ?? 0,
            angle_move_amplitude: res.parameters.angle_move_amplitude ?? 0,
          });
          sendStatus("Walking Params Loaded");
        }
      });
    }
  };

  const applyWalkingParams = () => {
    if (walkingParamPubRef.current && walkingFull) {
      const payload = { ...walkingFull, ...walkingParams };
      // ensure numbers
      payload.x_move_amplitude = Number(payload.x_move_amplitude);
      payload.y_move_amplitude = Number(payload.y_move_amplitude);
      payload.angle_move_amplitude = Number(payload.angle_move_amplitude);
      walkingParamPubRef.current.publish(new ROSLIB.Message(payload));
      sendStatus("Walking Params Applied");
    }
  };

  const applyYoloParams = () => {
    if (yoloSetServiceRef.current) {
      const parameters = YOLO_PARAM_KEYS.map(name => ({
        name,
        value: makeParamValue("double", yoloParams[name])
      }));
      yoloSetServiceRef.current.callService(new ROSLIB.ServiceRequest({ parameters }), (res) => {
        if (res?.results) sendStatus("YOLO Params Applied");
        else sendStatus("YOLO Update Failed", true);
      });
    }
  };

  // --- Derived Data ---
  const battery = metrics?.battery || {};
  const system = metrics?.system || {};
  const network = metrics?.network || {};
  const imu = metrics?.imu || {};
  const torque = metrics?.torque || {};
  const healthOk = healthResult?.ok || [];
  const healthFailed = healthResult?.failed || [];
  const healthErrors = healthResult?.errors || {};
  const healthTotal = healthResult?.total ?? (healthOk.length + healthFailed.length);
  const healthDuration = healthResult?.duration_ms;
  const healthSkipped = healthResult?.skipped;
  const healthRaw = healthResult?.raw;
  const torqueEntries = useMemo(() => {
    const e = Object.entries(torque?.joints || {});
    e.sort((a, b) => Math.abs(b[1]) - Math.abs(a[1]));
    return e.slice(0, 8);
  }, [torque]);

  // --- Render Views ---

  const renderSidebar = () => (
    <nav className="w-64 bg-sidebar flex flex-col p-6 text-gray-400 flex-shrink-0">
      <div className="flex items-center gap-3 mb-8 px-2">
        <img src="/log.png" alt="Bascorro Logo" className="w-8 h-8 object-contain" />
        <span className="font-display font-bold text-lg text-white tracking-tight">Bascorro</span>
      </div>
      <div className="flex flex-col gap-2 flex-1">
        <button
          className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "dashboard" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
          onClick={() => setActiveTab("dashboard")}
        >
          <LayoutDashboard size={20} className={activeTab === "dashboard" ? "text-accent-yellow" : ""} />
          <span>Dashboard</span>
        </button>
        <button
          className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "charts" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
          onClick={() => setActiveTab("charts")}
        >
          <Activity size={20} className={activeTab === "charts" ? "text-accent-yellow" : ""} />
          <span>Charts</span>
        </button>
        <button
          className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "vision" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
          onClick={() => setActiveTab("vision")}
        >
          <Video size={20} className={activeTab === "vision" ? "text-accent-yellow" : ""} />
          <span>Vision</span>
        </button>
        <button
          className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "tuning" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
          onClick={() => setActiveTab("tuning")}
        >
          <Settings size={20} className={activeTab === "tuning" ? "text-accent-yellow" : ""} />
          <span>Tuning</span>
        </button>
        <button
          className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "action" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
          onClick={() => setActiveTab("action")}
        >
          <Activity size={20} className={activeTab === "action" ? "text-accent-yellow" : ""} />
          <span>Action</span>
        </button>
        <button
          className={`flex items-center gap-3 px-4 py-3 rounded-lg font-medium transition-all ${activeTab === "logs" ? "bg-undip-blue text-white shadow-md border border-accent-yellow/20" : "hover:bg-white/5 hover:text-white"}`}
          onClick={() => setActiveTab("logs")}
        >
          <Terminal size={20} className={activeTab === "logs" ? "text-accent-yellow" : ""} />
          <span>Logs</span>
        </button>
      </div>
      <div className="pt-6 border-t border-white/10">
        <div className={`flex items-center gap-2 px-2 text-sm font-medium ${rosState === 'connected' ? 'text-green-400' : 'text-red-400'}`}>
          <div className={`w-2 h-2 rounded-full ${rosState === 'connected' ? 'bg-green-400 shadow-[0_0_8px_rgba(74,222,128,0.5)]' : 'bg-red-400'}`}></div>
          <span>{rosState === "connected" ? "System Online" : "Disconnected"}</span>
        </div>
      </div>
    </nav>
  );

  const renderDashboard = () => (
    <div className="grid grid-cols-[3fr_1fr] gap-6 h-full p-8 overflow-y-auto">
      <div className="flex flex-col gap-6">
        {/* Safety & Quick Actions */}
        <div className="flex gap-4 items-center bg-white p-4 rounded-2xl border border-gray-200 shadow-sm">
          <button className="flex items-center gap-2 px-5 py-2.5 bg-red-50 text-red-600 border border-red-100 rounded-lg font-bold hover:bg-red-600 hover:text-white transition-all shadow-sm" onClick={handleInitPose}>
            <RefreshCw size={18} /> Init Pose
          </button>
          <button className="flex items-center gap-2 px-5 py-2.5 bg-red-50 text-red-600 border border-red-100 rounded-lg font-bold hover:bg-red-600 hover:text-white transition-all shadow-sm" onClick={handleSoftStop}>
            <StopCircle size={18} /> Soft Stop
          </button>
          <div className="flex-1"></div>
          <div className="flex gap-2">
            <button className="px-4 py-2 border border-gray-200 rounded-lg text-gray-600 font-medium hover:bg-gray-50" onClick={() => handleTorque(false)}>Torque OFF</button>
            <button className="px-4 py-2 bg-undip-blue text-white rounded-lg font-bold hover:bg-opacity-90 shadow-sm transition-all" onClick={() => handleTorque(true)}>Torque ON</button>
          </div>
        </div>

        {/* Stats Grid */}
        <div className="grid grid-cols-4 gap-6">
          <StatCard
            title="Battery"
            icon={Battery}
            value={`${formatNumber(battery.voltage, 2)} V`}
            subValue={formatPercent(battery.percent)}
            status={battery.voltage < battery.warn_voltage ? "danger" : "neutral"}
          />
          <StatCard
            title="CPU Load"
            icon={Cpu}
            value={formatPercent(system.cpu_percent)}
            subValue={`Load: ${formatNumber(system.load, 2)}`}
          />
          <StatCard
            title="Memory"
            icon={HardDrive}
            value={formatPercent(system.mem_percent)}
            subValue={`${formatNumber(system.mem_used_mb, 0)} MB Used`}
          />
          <StatCard
            title="Network"
            icon={Wifi}
            value={`${formatNumber(network.latency_ms, 0)} ms`}
            subValue="Latency"
          />
        </div>

        {/* Torque Graph */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Joint Torque Load" />
          <div className="grid grid-cols-4 gap-4 mt-4">
            {torqueEntries.length === 0 ? <p className="text-gray-400 text-sm col-span-4 text-center py-8">No torque data available</p> :
              torqueEntries.map(([name, val]) => (
                <div key={name} className="flex flex-col gap-1">
                  <span className="text-xs font-medium text-gray-500 font-mono uppercase">{name}</span>
                  <div className="h-2 bg-gray-100 rounded-full overflow-hidden">
                    <div
                      className={`h-full rounded-full transition-all duration-300 ${Math.abs(val) > 80 ? 'bg-red-500' : 'bg-undip-blue'}`}
                      style={{ width: `${Math.min(Math.abs(val) * 10, 100)}%` }}
                    ></div>
                  </div>
                  <span className="text-xs font-bold text-gray-700 text-right">{formatNumber(val, 2)}</span>
                </div>
              ))
            }
          </div>
        </div>
      </div>

      <div className="flex flex-col gap-6">
        {/* IMU Card */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="IMU Orientation" />
          <div className="flex flex-col gap-4">
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500">Roll</span>
              <strong className="font-mono text-gray-900">{formatNumber(imu.roll, 1)}°</strong>
            </div>
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500">Pitch</span>
              <strong className="font-mono text-gray-900">{formatNumber(imu.pitch, 1)}°</strong>
            </div>
            <div className="flex justify-between items-center py-2 border-b border-gray-50">
              <span className="text-sm text-gray-500">Yaw</span>
              <strong className="font-mono text-gray-900">{formatNumber(imu.yaw, 1)}°</strong>
            </div>
            <div className={`mt-4 text-center p-2 rounded-lg text-xs font-bold uppercase tracking-wider ${metrics?.fall?.state !== "upright" ? "bg-red-100 text-red-600" : "bg-green-50 text-green-600"}`}>
              State: {metrics?.fall?.state || "Unknown"}
            </div>
          </div>
        </div>

        {/* Demo Control */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Demo Control" />
          <div className="flex flex-col gap-3">
            <div className="grid grid-cols-4 gap-2">
              {["ready", "soccer", "vision", "action"].map((mode) => (
                <button
                  key={mode}
                  className={`py-2 rounded-lg text-[10px] font-bold uppercase transition-all border disabled:opacity-50 disabled:cursor-not-allowed ${demoMode === mode ? "bg-undip-blue text-white border-undip-blue" : "bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100"}`}
                  onClick={() => handleDemoMode(mode)}
                  disabled={rosState !== "connected"}
                >
                  {mode}
                </button>
              ))}
            </div>
            <div className="grid grid-cols-2 gap-2">
              <button
                className="py-2 rounded-lg text-xs font-bold bg-green-50 text-green-700 border border-green-100 hover:bg-green-100 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                onClick={() => handleDemoCommand("start")}
                disabled={rosState !== "connected"}
              >
                Start
              </button>
              <button
                className="py-2 rounded-lg text-xs font-bold bg-red-50 text-red-600 border border-red-100 hover:bg-red-100 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                onClick={() => handleDemoCommand("stop")}
                disabled={rosState !== "connected"}
              >
                Stop
              </button>
            </div>
            <div className="text-xs text-gray-500 font-mono">
              {demoCommandAt ? `${new Date(demoCommandAt).toLocaleTimeString()} | ${demoCommand}` : "No demo command sent"}
            </div>
            <div className="text-[11px] text-gray-400">
              Start/Stop affects soccer and action demos.
            </div>
          </div>
        </div>

        {/* Joint Health */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Joint Health">
            <button
              className="flex items-center gap-2 px-3 py-1.5 rounded-lg text-xs font-bold bg-gray-100 text-gray-700 hover:bg-gray-200 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
              onClick={handleHealthCheck}
              disabled={healthRunning || rosState !== "connected"}
            >
              <AlertCircle size={14} />
              {healthRunning ? "Checking..." : "Run Check"}
            </button>
          </SectionHeader>
          {healthCheckedAt && (
            <div className="text-xs text-gray-400 font-mono mb-3">
              Last check: {new Date(healthCheckedAt).toLocaleTimeString()}
            </div>
          )}
          {healthError && (
            <div className="mb-3 text-xs text-red-600 font-mono">{healthError}</div>
          )}
          {!healthResult && !healthError && (
            <div className="text-sm text-gray-400">No health check yet</div>
          )}
          {healthResult && !healthError && (
            <div className="flex flex-col gap-3">
              <div className={`px-3 py-2 rounded-lg text-xs font-bold uppercase tracking-wider ${healthSkipped ? "bg-gray-100 text-gray-500" : healthFailed.length > 0 ? "bg-red-50 text-red-600" : "bg-green-50 text-green-600"}`}>
                {healthSkipped ? "Skipped (simulation)" : healthFailed.length > 0 ? `${healthFailed.length} joints failed` : "All joints responded"}
              </div>
              <div className="flex justify-between text-xs text-gray-500 font-mono">
                <span>{healthTotal} joints</span>
                <span>{healthDuration !== undefined ? `${formatNumber(healthDuration, 1)} ms` : "-"}</span>
              </div>
              {healthFailed.length > 0 ? (
                <div className="max-h-[180px] overflow-y-auto pr-2 custom-scrollbar space-y-2">
                  {healthFailed.map((name) => (
                    <div key={name} className="flex justify-between text-xs">
                      <span className="font-mono text-gray-700">{name}</span>
                      <span className="text-red-600">{healthErrors[name] || "No response"}</span>
                    </div>
                  ))}
                </div>
              ) : (
                <div className="text-sm text-gray-500">No failures reported.</div>
              )}
              {healthRaw && (
                <div className="text-xs text-gray-400 font-mono break-words">{healthRaw}</div>
              )}
            </div>
          )}
        </div>

        {/* Logs Preview */}
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm flex-1">
          <SectionHeader title="System Events" />
          <div className="flex flex-col gap-3 mt-2 overflow-y-auto max-h-[300px] pr-2 custom-scrollbar">
            {events.slice().reverse().slice(0, 10).map((ev, i) => (
              <div key={i} className="flex gap-3 text-xs pb-3 border-b border-gray-50 last:border-0">
                <span className="font-mono text-gray-400 whitespace-nowrap">{new Date(ev.ts * 1000).toLocaleTimeString([], {hour:'2-digit', minute:'2-digit', second:'2-digit'})}</span>
                <span className="text-gray-700">{ev.message}</span>
              </div>
            ))}
            {events.length === 0 && <p className="text-gray-400 text-sm text-center py-8">No events logged</p>}
          </div>
        </div>
      </div>
    </div>
  );

  const renderVision = () => (
    <div className="grid grid-cols-[1fr_320px] gap-6 h-full p-8 overflow-hidden">
      <div className="bg-black rounded-2xl overflow-hidden relative flex items-center justify-center border border-gray-800 shadow-lg">
        {showOverlay ? <canvas ref={overlayCanvasRef} className="max-w-full max-h-full object-contain" /> : <div className="text-gray-500 text-sm font-mono">Stream Paused</div>}
        <div className="absolute top-4 right-4 bg-black/70 text-green-400 px-3 py-1 rounded-full text-xs font-mono backdrop-blur-sm border border-white/10 flex items-center gap-2">
          <div className="w-2 h-2 rounded-full bg-green-400 animate-pulse"></div>
          {formatNumber(overlayStats.fps, 1)} FPS
        </div>
      </div>
      <div className="flex flex-col gap-6 overflow-y-auto pr-2 custom-scrollbar">
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Stream Control" />
          <div className="flex flex-col gap-4">
            <div>
              <label className="block text-xs font-bold text-gray-500 uppercase tracking-wider mb-2">Preset Topics</label>
              <div className="grid grid-cols-2 gap-2">
                <button 
                  onClick={() => setOverlayTopic("/vision/yolo/debug")} 
                  className={`py-2 rounded-lg text-[10px] font-bold transition-all border ${overlayTopic === "/vision/yolo/debug" ? 'bg-undip-blue text-white border-undip-blue' : 'bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100'}`}
                >
                  YOLO Debug
                </button>
                <button 
                  onClick={() => setOverlayTopic("/robotis_op3/camera/image_raw")} 
                  className={`py-2 rounded-lg text-[10px] font-bold transition-all border ${overlayTopic === "/robotis_op3/camera/image_raw" ? 'bg-undip-blue text-white border-undip-blue' : 'bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100'}`}
                >
                  Pure Camera
                </button>
              </div>
            </div>
            <div>
              <label className="block text-xs font-bold text-gray-500 uppercase tracking-wider mb-2">Custom Topic</label>
              <input type="text" className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:outline-none focus:ring-2 focus:ring-undip-blue/20 focus:border-undip-blue" value={overlayTopic} onChange={e => setOverlayTopic(e.target.value)} />
            </div>
            <div className="flex gap-2">
              <button className="flex-1 py-2 bg-gray-100 hover:bg-gray-200 text-gray-700 rounded-lg text-sm font-bold transition-colors" onClick={() => setShowOverlay(!showOverlay)}>
                {showOverlay ? "Stop Stream" : "Start Stream"}
              </button>
              <button className="flex-1 py-2 bg-undip-blue text-white hover:bg-opacity-90 rounded-lg text-sm font-bold transition-colors flex items-center justify-center gap-2" onClick={() => snapshotServiceRef.current?.callService({}, () => sendStatus("Snapshot Saved"))}>
                <Camera size={14} /> Snap
              </button>
            </div>
          </div>
        </div>
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="YOLO Thresholds" />
          <div className="flex flex-col gap-6 mt-2">
            {YOLO_PARAM_KEYS.map(key => (
              <div key={key}>
                <div className="flex justify-between mb-2">
                  <label className="text-xs font-bold text-gray-500 uppercase tracking-wider">{key.replace(/_confidence_threshold|_/g, " ")}</label>
                  <span className="text-xs font-mono font-bold text-undip-blue">{yoloParams[key].toFixed(2)}</span>
                </div>
                <input
                  type="range" min="0" max="1" step="0.05"
                  className="w-full accent-undip-blue h-2 bg-gray-100 rounded-lg appearance-none cursor-pointer"
                  value={yoloParams[key]}
                  onChange={e => setYoloParams({ ...yoloParams, [key]: Number(e.target.value) })}
                />
              </div>
            ))}
            <button className="w-full py-3 bg-accent-yellow text-black hover:bg-yellow-400 rounded-lg text-sm font-bold transition-colors shadow-sm" onClick={applyYoloParams}>Apply Thresholds</button>
          </div>
        </div>
      </div>
    </div>
  );

  const renderTuning = () => (
    <div className="grid grid-cols-[1fr_340px] gap-6 h-full p-8 overflow-hidden">
      <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm flex flex-col h-full">
        <SectionHeader title="Teleoperation" />
        <div className="flex-1 flex items-center justify-center bg-gray-50 rounded-xl border border-gray-100">
          <GamepadVisualizer
            joy={joyState}
            rosConnected={rosState === "connected"}
            publishJoy={publishJoy}
          />
        </div>
      </div>
      <div className="flex flex-col gap-6 overflow-y-auto pr-2 custom-scrollbar">
        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <div className="flex justify-between items-center mb-4">
            <h2 className="text-lg font-bold font-display text-gray-800">Walking Params</h2>
            <button className="p-2 hover:bg-gray-100 rounded-lg text-gray-500 transition-colors" onClick={loadWalkingParams}><RefreshCw size={14}/></button>
          </div>
          <div className="flex flex-col gap-4">
            {['x_move_amplitude', 'y_move_amplitude', 'angle_move_amplitude'].map(param => (
              <div key={param}>
                <label className="block text-xs font-bold text-gray-500 uppercase tracking-wider mb-2">{param.replace(/_/g, ' ')}</label>
                <input 
                  type="number" 
                  step="0.001" 
                  className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:outline-none focus:ring-2 focus:ring-undip-blue/20 focus:border-undip-blue"
                  value={walkingParams[param]} 
                  onChange={e => setWalkingParams({...walkingParams, [param]: e.target.value})} 
                />
              </div>
            ))}
            <button className="w-full py-3 bg-undip-blue text-white hover:bg-opacity-90 rounded-lg text-sm font-bold transition-colors shadow-sm mt-2" onClick={applyWalkingParams}>Apply Params</button>
          </div>
        </div>

        <div className="bg-white p-6 rounded-2xl border border-gray-200 shadow-sm">
          <SectionHeader title="Manual Parameter" />
          <div className="flex flex-col gap-4">
            <div>
              <label className="block text-xs font-bold text-gray-500 uppercase tracking-wider mb-2">Node Name</label>
              <input type="text" className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:outline-none focus:ring-2 focus:ring-undip-blue/20 focus:border-undip-blue" value={paramNode} onChange={e => setParamNode(e.target.value)} />
            </div>
            <div>
              <label className="block text-xs font-bold text-gray-500 uppercase tracking-wider mb-2">Param Name</label>
              <input type="text" className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:outline-none focus:ring-2 focus:ring-undip-blue/20 focus:border-undip-blue" value={paramName} onChange={e => setParamName(e.target.value)} />
            </div>
            <div>
              <label className="block text-xs font-bold text-gray-500 uppercase tracking-wider mb-2">Value</label>
              <input type="text" className="w-full px-3 py-2 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:outline-none focus:ring-2 focus:ring-undip-blue/20 focus:border-undip-blue" value={paramValue} onChange={e => setParamValue(e.target.value)} />
            </div>
            <button className="w-full py-2 bg-gray-100 text-gray-700 hover:bg-gray-200 rounded-lg text-sm font-bold transition-colors mt-2" onClick={() => {
               const service = new ROSLIB.Service({ ros: rosRef.current, name: `/${paramNode}/set_parameters`, serviceType: "rcl_interfaces/srv/SetParameters" });
               service.callService(new ROSLIB.ServiceRequest({ parameters: [{ name: paramName, value: makeParamValue("double", paramValue) }] }), () => sendStatus("Param Sent"));
            }}>Set Value</button>
          </div>
        </div>
      </div>
    </div>
  );

  const renderLogs = () => (
    <div className="h-full p-8 overflow-hidden flex flex-col">
      <div className="bg-white rounded-2xl border border-gray-200 shadow-sm flex-1 flex flex-col overflow-hidden">
        <div className="p-6 border-b border-gray-100 flex justify-between items-center">
          <h2 className="text-lg font-bold font-display text-gray-800">System Logs</h2>
          <div className="flex gap-2">
             <span className="px-3 py-1 bg-gray-100 rounded-full text-xs font-medium text-gray-600">{events.length} Events</span>
          </div>
        </div>
        <div className="flex-1 overflow-y-auto p-0">
          <table className="w-full text-left border-collapse">
            <thead className="bg-gray-50 sticky top-0 z-10">
              <tr>
                <th className="py-3 px-6 text-xs font-bold text-gray-500 uppercase tracking-wider w-32 border-b border-gray-200">Time</th>
                <th className="py-3 px-6 text-xs font-bold text-gray-500 uppercase tracking-wider w-24 border-b border-gray-200">Type</th>
                <th className="py-3 px-6 text-xs font-bold text-gray-500 uppercase tracking-wider border-b border-gray-200">Message</th>
              </tr>
            </thead>
            <tbody className="divide-y divide-gray-100">
              {events.slice().reverse().map((ev, i) => (
                <tr key={i} className="hover:bg-gray-50 transition-colors group">
                  <td className="py-3 px-6 text-xs font-mono text-gray-500">{new Date(ev.ts * 1000).toLocaleTimeString()}</td>
                  <td className="py-3 px-6">
                    <span className={`inline-block px-2 py-0.5 rounded text-[10px] font-bold uppercase ${ev.type === 'error' ? 'bg-red-100 text-red-600' : ev.type === 'warn' ? 'bg-yellow-100 text-yellow-700' : 'bg-blue-50 text-blue-600'}`}>
                      {ev.type || "INFO"}
                    </span>
                  </td>
                  <td className="py-3 px-6 text-sm text-gray-700 font-mono">{ev.message}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );

  return (
    <div className="flex h-screen bg-gray-50 font-sans text-gray-900 overflow-hidden">
      {renderSidebar()}
      <main className="flex-1 flex flex-col min-w-0 bg-[#f8fafc]">
        <header className="h-16 bg-white border-b border-gray-200 flex items-center justify-between px-8 flex-shrink-0 z-20">
          <h1 className="text-xl font-bold font-display text-gray-900 capitalize tracking-tight">{activeTab}</h1>
          <div className={`px-4 py-1.5 rounded-full text-sm font-medium transition-all ${studioError ? "bg-red-50 text-red-600 border border-red-100" : "bg-green-50 text-green-600 border border-green-100"} ${!studioStatus && "opacity-0"}`}>
            {studioStatus || "Ready"}
          </div>
        </header>
        <div className="flex-1 overflow-hidden relative">
          {activeTab === "dashboard" && renderDashboard()}
          {activeTab === "charts" && <ChartPage currentMetrics={metrics} />}
          {activeTab === "vision" && renderVision()}
          {activeTab === "tuning" && renderTuning()}
          {activeTab === "logs" && renderLogs()}
          <div className={`absolute inset-0 p-4 ${activeTab === "action" ? "" : "hidden"}`}>
             <div className="bg-white rounded-2xl border border-gray-200 shadow-sm h-full overflow-hidden">
               <ActionEditor isActive={activeTab === "action"} />
             </div>
          </div>
        </div>
      </main>
    </div>
  );
}
