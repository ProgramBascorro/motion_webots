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

// --- Components ---

const StatCard = ({ title, icon: Icon, value, subValue, status = "neutral" }) => (
  <div className={`stat-card ${status}`}>
    <div className="stat-header">
      <span className="stat-title">{title}</span>
      {Icon && <Icon size={16} className="stat-icon" />}
    </div>
    <div className="stat-body">
      <span className="stat-value">{value}</span>
      {subValue && <span className="stat-sub">{subValue}</span>}
    </div>
  </div>
);

const SectionHeader = ({ title, children }) => (
  <div className="section-header">
    <h2>{title}</h2>
    <div className="section-actions">{children}</div>
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
  const torqueEntries = useMemo(() => {
    const e = Object.entries(torque?.joints || {});
    e.sort((a, b) => Math.abs(b[1]) - Math.abs(a[1]));
    return e.slice(0, 8);
  }, [torque]);

  // --- Render Views ---

  const renderSidebar = () => (
    <nav className="sidebar">
      <div className="sidebar-brand">
        <div className="brand-logo">B</div>
        <span className="brand-name">Bascorro</span>
      </div>
      <div className="sidebar-menu">
        <button
          className={activeTab === "dashboard" ? "active" : ""}
          onClick={() => setActiveTab("dashboard")}
        >
          <LayoutDashboard size={20} />
          <span>Dashboard</span>
        </button>
        <button
          className={activeTab === "vision" ? "active" : ""}
          onClick={() => setActiveTab("vision")}
        >
          <Video size={20} />
          <span>Vision</span>
        </button>
        <button
          className={activeTab === "tuning" ? "active" : ""}
          onClick={() => setActiveTab("tuning")}
        >
          <Settings size={20} />
          <span>Tuning</span>
        </button>
        <button
          className={activeTab === "action" ? "active" : ""}
          onClick={() => setActiveTab("action")}
        >
          <Activity size={20} />
          <span>Action</span>
        </button>
        <button
          className={activeTab === "logs" ? "active" : ""}
          onClick={() => setActiveTab("logs")}
        >
          <Terminal size={20} />
          <span>Logs</span>
        </button>
      </div>
      <div className="sidebar-footer">
        <div className={`connection-status ${rosState}`}>
          <div className="status-dot"></div>
          <span>{rosState === "connected" ? "Online" : "Offline"}</span>
        </div>
      </div>
    </nav>
  );

  const renderDashboard = () => (
    <div className="view-content dashboard-grid">
      <div className="dashboard-col-main">
        <div className="safety-bar">
          <button className="panic-btn init-pose" onClick={handleInitPose}>
            <RefreshCw size={18} /> Init Pose
          </button>
          <button className="panic-btn soft-stop" onClick={handleSoftStop}>
            <StopCircle size={18} /> Soft Stop
          </button>
          <div className="spacer"></div>
          <button className="control-btn" onClick={() => handleTorque(false)}>Torque OFF</button>
          <button className="control-btn primary" onClick={() => handleTorque(true)}>Torque ON</button>
        </div>

        <div className="stats-row">
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

        <div className="card full-width">
          <SectionHeader title="Joint Torque Load" />
          <div className="torque-grid">
            {torqueEntries.length === 0 ? <p className="empty-text">No torque data</p> :
              torqueEntries.map(([name, val]) => (
                <div key={name} className="torque-item">
                  <span className="joint-name">{name}</span>
                  <div className="torque-bar-bg">
                    <div
                      className="torque-bar-fill"
                      style={{ width: `${Math.min(Math.abs(val) * 10, 100)}%` }}
                    ></div>
                  </div>
                  <span className="torque-val">{formatNumber(val, 2)}</span>
                </div>
              ))
            }
          </div>
        </div>
      </div>

      <div className="dashboard-col-side">
        <div className="card">
          <SectionHeader title="IMU State" />
          <div className="imu-readout">
            <div className="imu-row">
              <span>Roll</span>
              <strong>{formatNumber(imu.roll, 1)}°</strong>
            </div>
            <div className="imu-row">
              <span>Pitch</span>
              <strong>{formatNumber(imu.pitch, 1)}°</strong>
            </div>
            <div className="imu-row">
              <span>Yaw</span>
              <strong>{formatNumber(imu.yaw, 1)}°</strong>
            </div>
            <div className={`fall-status ${metrics?.fall?.state !== "upright" ? "fallen" : ""}`}>
              {metrics?.fall?.state || "Unknown"}
            </div>
          </div>
        </div>

        <div className="card">
          <SectionHeader title="Recent Events" />
          <div className="mini-events">
            {events.slice().reverse().slice(0, 5).map((ev, i) => (
              <div key={i} className="mini-event">
                <span className="time">{new Date(ev.ts * 1000).toLocaleTimeString([], {hour:'2-digit', minute:'2-digit', second:'2-digit'})}</span>
                <span className="msg">{ev.message}</span>
              </div>
            ))}
            {events.length === 0 && <p className="empty-text">No events</p>}
          </div>
        </div>
      </div>
    </div>
  );

  const renderVision = () => (
    <div className="view-content vision-layout">
      <div className="vision-stream-container">
        {showOverlay ? <canvas ref={overlayCanvasRef} className="vision-canvas" /> : <div className="vision-placeholder">Stream Paused</div>}
        <div className="vision-overlay-stats">
          {formatNumber(overlayStats.fps, 1)} FPS
        </div>
      </div>
      <div className="vision-sidebar">
        <div className="card">
          <SectionHeader title="Controls" />
          <div className="form-group">
            <label>Topic</label>
            <input type="text" value={overlayTopic} onChange={e => setOverlayTopic(e.target.value)} />
          </div>
          <div className="btn-group">
            <button className="control-btn" onClick={() => setShowOverlay(!showOverlay)}>
              {showOverlay ? "Pause" : "Resume"}
            </button>
            <button className="control-btn" onClick={() => snapshotServiceRef.current?.callService({}, () => sendStatus("Snapshot Saved"))}>
              <Camera size={16} /> Snapshot
            </button>
          </div>
        </div>
        <div className="card">
          <SectionHeader title="YOLO Thresholds" />
          {YOLO_PARAM_KEYS.map(key => (
            <div key={key} className="range-control">
              <label>{key.replace(/_confidence_threshold|_/g, " ")}</label>
              <div className="range-row">
                <input
                  type="range" min="0" max="1" step="0.05"
                  value={yoloParams[key]}
                  onChange={e => setYoloParams({ ...yoloParams, [key]: Number(e.target.value) })}
                />
                <span>{yoloParams[key].toFixed(2)}</span>
              </div>
            </div>
          ))}
          <button className="control-btn primary" onClick={applyYoloParams}>Apply Thresholds</button>
        </div>
      </div>
    </div>
  );

  const renderTuning = () => (
    <div className="view-content tuning-grid">
      <div className="card full-height">
        <SectionHeader title="Teleoperation" />
        <div className="gamepad-wrapper">
          <GamepadVisualizer
            joy={joyState}
            rosConnected={rosState === "connected"}
            publishJoy={publishJoy}
          />
        </div>
      </div>
      <div className="tuning-col">
        <div className="card">
          <SectionHeader title="Walking Parameters">
            <button className="icon-btn" onClick={loadWalkingParams}><RefreshCw size={14}/></button>
          </SectionHeader>
          <div className="form-group">
            <label>X Amplitude</label>
            <input type="number" step="0.001" value={walkingParams.x_move_amplitude} onChange={e => setWalkingParams({...walkingParams, x_move_amplitude: e.target.value})} />
          </div>
          <div className="form-group">
            <label>Y Amplitude</label>
            <input type="number" step="0.001" value={walkingParams.y_move_amplitude} onChange={e => setWalkingParams({...walkingParams, y_move_amplitude: e.target.value})} />
          </div>
          <div className="form-group">
            <label>Angle Amplitude</label>
            <input type="number" step="0.001" value={walkingParams.angle_move_amplitude} onChange={e => setWalkingParams({...walkingParams, angle_move_amplitude: e.target.value})} />
          </div>
          <button className="control-btn primary" onClick={applyWalkingParams}>Apply Walking Params</button>
        </div>

        <div className="card">
          <SectionHeader title="Manual Parameter" />
          <div className="form-group">
            <label>Node</label>
            <input type="text" value={paramNode} onChange={e => setParamNode(e.target.value)} />
          </div>
          <div className="form-group">
            <label>Param Name</label>
            <input type="text" value={paramName} onChange={e => setParamName(e.target.value)} />
          </div>
          <div className="form-group">
            <label>Value</label>
            <input type="text" value={paramValue} onChange={e => setParamValue(e.target.value)} />
          </div>
          {/* Simplified type selector for brevity */}
          <button className="control-btn" onClick={() => {
             const service = new ROSLIB.Service({ ros: rosRef.current, name: `/${paramNode}/set_parameters`, serviceType: "rcl_interfaces/srv/SetParameters" });
             service.callService(new ROSLIB.ServiceRequest({ parameters: [{ name: paramName, value: makeParamValue("double", paramValue) }] }), () => sendStatus("Param Sent"));
          }}>Set (Double)</button>
        </div>
      </div>
    </div>
  );

  const renderLogs = () => (
    <div className="view-content">
      <div className="card full-height">
        <SectionHeader title="System Logs" />
        <div className="logs-table-container">
          <table className="logs-table">
            <thead>
              <tr>
                <th>Time</th>
                <th>Type</th>
                <th>Message</th>
              </tr>
            </thead>
            <tbody>
              {events.slice().reverse().map((ev, i) => (
                <tr key={i} className={`log-row ${ev.type || "info"}`}>
                  <td className="log-time">{new Date(ev.ts * 1000).toLocaleTimeString()}</td>
                  <td className="log-type">{ev.type || "INFO"}</td>
                  <td className="log-msg">{ev.message}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );

  return (
    <div className="app-container">
      {renderSidebar()}
      <main className="main-content">
        <header className="top-bar">
          <h1>{activeTab.charAt(0).toUpperCase() + activeTab.slice(1)}</h1>
          <div className="status-toast">{studioStatus}</div>
        </header>
        {activeTab === "dashboard" && renderDashboard()}
        {activeTab === "vision" && renderVision()}
        {activeTab === "tuning" && renderTuning()}
        {activeTab === "logs" && renderLogs()}
        {activeTab === "action" && <div className="view-content"><ActionEditor /></div>}
      </main>
    </div>
  );
}