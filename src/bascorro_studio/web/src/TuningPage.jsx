import { useEffect, useState, useMemo, useRef } from "react";
import ROSLIB from "roslib";
import {
  RefreshCw,
  Zap,
  Save,
  Play,
  Square,
  Search,
  AlertCircle,
  Sliders,
  Cpu,
  Gamepad2,
  Terminal,
  CheckCircle2,
  Download,
} from "lucide-react";
import GamepadVisualizer from "./GamepadVisualizer.jsx";

// --- Constants ---

const PARAM_TYPES = {
  bool: 1,
  int: 2,
  double: 3,
  string: 4,
};

const WALKING_PARAM_FIELDS = [
  {
    key: "x_move_amplitude",
    step: "0.001",
    label: "X Amplitude",
    quick: [
      { label: "-0.005", delta: -0.005 },
      { label: "+0.005", delta: 0.005 },
    ],
  },
  {
    key: "y_move_amplitude",
    step: "0.001",
    label: "Y Amplitude",
    quick: [
      { label: "-0.005", delta: -0.005 },
      { label: "+0.005", delta: 0.005 },
    ],
  },
  {
    key: "angle_move_amplitude",
    step: "0.001",
    label: "Angle Amplitude",
    quick: [
      { label: "-0.01", delta: -0.01 },
      { label: "+0.01", delta: 0.01 },
    ],
  },
  {
    key: "period_time",
    step: "0.001",
    label: "Period (s)",
    quick: [
      { label: "-0.01s", delta: -0.01 },
      { label: "+0.01s", delta: 0.01 },
    ],
  },
];

const JOINT_GROUPS = {
  "Head": ["head"],
  "Arms": ["sho", "el"],
  "Legs": ["hip", "knee", "ank"],
};

// --- Helpers ---

function radToDeg(value) {
  const num = Number(value);
  if (!Number.isFinite(num)) return 0;
  return num * (180 / Math.PI);
}

function degToRad(value) {
  const num = Number(value);
  if (!Number.isFinite(num)) return 0;
  return num * (Math.PI / 180);
}

function toNumber(value, fallback = 0) {
  const num = Number(value);
  return Number.isFinite(num) ? num : fallback;
}

function makeParamValue(type, rawValue) {
  const value = String(rawValue ?? "").trim();
  if (type === "bool") return { type: PARAM_TYPES.bool, bool_value: value === "true" };
  if (type === "int") return { type: PARAM_TYPES.int, integer_value: Number(value) };
  if (type === "string") return { type: PARAM_TYPES.string, string_value: value };
  return { type: PARAM_TYPES.double, double_value: Number(value) };
}

// --- Components ---

const SectionCard = ({ title, icon: Icon, children, actions, className = "" }) => (
  <div className={`bg-white rounded-2xl border border-gray-200 shadow-sm flex flex-col overflow-hidden ${className}`}>
    <div className="px-5 py-4 border-b border-gray-100 flex justify-between items-center bg-gray-50/50">
      <div className="flex items-center gap-2">
        {Icon && <Icon size={18} className="text-undip-blue" />}
        <h2 className="text-sm font-bold font-display text-gray-800 uppercase tracking-wide">{title}</h2>
      </div>
      <div className="flex gap-2">{actions}</div>
    </div>
    <div className="p-5 flex-1 overflow-auto custom-scrollbar">
      {children}
    </div>
  </div>
);

const NumberInput = ({ value, onChange, step = 1, className = "" }) => (
  <input
    type="number"
    step={step}
    value={value}
    onChange={onChange}
    className={`w-full px-2 py-1.5 bg-gray-50 border border-gray-200 rounded-lg text-xs font-mono font-medium focus:outline-none focus:border-undip-blue focus:ring-2 focus:ring-undip-blue/10 transition-all ${className}`}
  />
);

export default function TuningPage({ ros, rosState, joyState, publishJoy, sendStatus }) {
  // --- State: Offsets ---
  const [offsetRows, setOffsetRows] = useState([]);
  const [offsetLoading, setOffsetLoading] = useState(false);
  const [offsetError, setOffsetError] = useState("");
  const [offsetFilter, setOffsetFilter] = useState("");
  const [activeGroup, setActiveGroup] = useState("All");

  // --- State: Walking ---
  const [walkingParams, setWalkingParams] = useState({
    x_move_amplitude: 0.0,
    y_move_amplitude: 0.0,
    angle_move_amplitude: 0.0,
    period_time: 0.0,
  });
  const [walkingFull, setWalkingFull] = useState(null);
  
  // --- State: Manual Param ---
  const [paramNode, setParamNode] = useState("op3_yolo_vision");
  const [paramName, setParamName] = useState("ball_confidence_threshold");
  const [paramValue, setParamValue] = useState("0.2");

  // --- Refs ---
  const offsetDataPubRef = useRef(null);
  const offsetTorquePubRef = useRef(null);
  const offsetCommandPubRef = useRef(null);
  const offsetServiceRef = useRef(null);
  
  const walkingCommandPubRef = useRef(null);
  const walkingParamPubRef = useRef(null);
  const walkingGetServiceRef = useRef(null);
  const enableModulePubRef = useRef(null);

  // --- Setup ROS ---
  useEffect(() => {
    if (!ros) return;

    // Offset Tuner
    offsetDataPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/offset_tuner/joint_offset_data",
      messageType: "op3_offset_tuner_msgs/JointOffsetData",
    });
    offsetTorquePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/offset_tuner/torque_enable",
      messageType: "op3_offset_tuner_msgs/JointTorqueOnOffArray",
    });
    offsetCommandPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/offset_tuner/command",
      messageType: "std_msgs/String",
    });
    offsetServiceRef.current = new ROSLIB.Service({
      ros,
      name: "/robotis/offset_tuner/get_present_joint_offset_data",
      serviceType: "op3_offset_tuner_msgs/srv/GetPresentJointOffsetData",
    });

    // Walking
    walkingCommandPubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/walking/command",
      messageType: "std_msgs/String",
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
    enableModulePubRef.current = new ROSLIB.Topic({
      ros,
      name: "/robotis/enable_ctrl_module",
      messageType: "std_msgs/String",
    });

  }, [ros]);

  // --- Actions: Offsets ---

  const loadOffsetRows = () => {
    if (!offsetServiceRef.current || rosState !== "connected") {
      sendStatus("Offset tuner service unavailable", true);
      return;
    }
    setOffsetLoading(true);
    setOffsetError("");
    offsetServiceRef.current.callService(new ROSLIB.ServiceRequest({}), (res) => {
      const rows = (res?.present_data_array || []).map((item) => ({
        joint_name: item.joint_name,
        goal_deg: radToDeg(item.goal_value ?? 0),
        offset_deg: radToDeg(item.offset_value ?? 0),
        present_deg: radToDeg(item.present_value ?? 0),
        p_gain: item.p_gain ?? 0,
        i_gain: item.i_gain ?? 0,
        d_gain: item.d_gain ?? 0,
        torque_enable: true,
      }));
      setOffsetRows(rows);
      setOffsetLoading(false);
      sendStatus("Offset data loaded");
    }, () => {
      setOffsetError("Failed to load offset data");
      setOffsetLoading(false);
      sendStatus("Offset load failed", true);
    });
  };

  const updateOffsetRow = (jointName, field, value) => {
    setOffsetRows((prev) => prev.map((row) => (
      row.joint_name === jointName ? { ...row, [field]: value } : row
    )));
  };

  const publishOffsetRow = (row) => {
    if (!offsetDataPubRef.current || rosState !== "connected") return false;
    offsetDataPubRef.current.publish(new ROSLIB.Message({
      joint_name: row.joint_name,
      goal_value: degToRad(toNumber(row.goal_deg)),
      offset_value: degToRad(toNumber(row.offset_deg)),
      p_gain: Math.round(toNumber(row.p_gain)),
      i_gain: Math.round(toNumber(row.i_gain)),
      d_gain: Math.round(toNumber(row.d_gain)),
    }));
    return true;
  };

  const applyAllOffsets = () => {
    let sent = 0;
    offsetRows.forEach((row) => { if (publishOffsetRow(row)) sent += 1; });
    if (sent > 0) sendStatus(`Sent offsets: ${sent} joints`);
  };

  const setOffsetTorque = (jointName, enable) => {
    if (!offsetTorquePubRef.current || rosState !== "connected") return;
    offsetTorquePubRef.current.publish(new ROSLIB.Message({
      torque_enable_data: [{ joint_name: jointName, torque_enable: enable }],
    }));
    setOffsetRows((prev) => prev.map((row) => (
      row.joint_name === jointName ? { ...row, torque_enable: enable } : row
    )));
  };

  const setOffsetTorqueAll = (enable) => {
    if (!offsetTorquePubRef.current || rosState !== "connected" || offsetRows.length === 0) return;
    offsetTorquePubRef.current.publish(new ROSLIB.Message({
      torque_enable_data: offsetRows.map((row) => ({ joint_name: row.joint_name, torque_enable: enable })),
    }));
    setOffsetRows((prev) => prev.map((row) => ({ ...row, torque_enable: enable })));
    sendStatus(`Torque ${enable ? "ON" : "OFF"} all`);
  };

  const sendOffsetCommand = (command, label) => {
    if (!offsetCommandPubRef.current || rosState !== "connected") return;
    offsetCommandPubRef.current.publish(new ROSLIB.Message({ data: command }));
    sendStatus(label || `Command: ${command}`);
  };

  const downloadOffsetYAML = () => {
    if (offsetRows.length === 0) {
      sendStatus("No offset data to download", true);
      return;
    }

    // Build YAML content
    let yamlContent = "offset:\n";
    offsetRows.forEach(row => {
      yamlContent += `  ${row.joint_name}: ${toNumber(row.offset_deg) !== 0 ? degToRad(toNumber(row.offset_deg)).toFixed(6) : 0}\n`;
    });

    yamlContent += "init_pose_for_offset_tuner:\n";
    offsetRows.forEach(row => {
      yamlContent += `  ${row.joint_name}: ${toNumber(row.goal_deg) !== 0 ? degToRad(toNumber(row.goal_deg)).toFixed(6) : 0}\n`;
    });

    // Create and download file
    const blob = new Blob([yamlContent], { type: 'text/yaml' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, -5);
    a.download = `offset_${timestamp}.yaml`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    sendStatus("Offset YAML downloaded");
  };

  // --- Actions: Walking ---

  const loadWalkingParams = () => {
    if (!walkingGetServiceRef.current || rosState !== "connected") return;
    walkingGetServiceRef.current.callService(new ROSLIB.ServiceRequest({ get_param: true }), (res) => {
      if (res?.parameters) {
        setWalkingFull(res.parameters);
        setWalkingParams({
          x_move_amplitude: res.parameters.x_move_amplitude ?? 0,
          y_move_amplitude: res.parameters.y_move_amplitude ?? 0,
          angle_move_amplitude: res.parameters.angle_move_amplitude ?? 0,
          period_time: res.parameters.period_time ?? 0,
        });
        sendStatus("Walking Params Loaded");
      }
    });
  };

  const applyWalkingParams = () => {
    if (!walkingParamPubRef.current || !walkingFull) {
      sendStatus("Load params first", true);
      return false;
    }
    const payload = { ...walkingFull, ...walkingParams };
    // Ensure numbers
    payload.x_move_amplitude = Number(payload.x_move_amplitude);
    payload.y_move_amplitude = Number(payload.y_move_amplitude);
    payload.angle_move_amplitude = Number(payload.angle_move_amplitude);
    payload.period_time = Number(payload.period_time);
    walkingParamPubRef.current.publish(new ROSLIB.Message(payload));
    sendStatus("Walking Params Applied");
    return true;
  };

  const sendWalkingCommand = (command, label) => {
    if (!walkingCommandPubRef.current || rosState !== "connected") return;
    walkingCommandPubRef.current.publish(new ROSLIB.Message({ data: command }));
    sendStatus(label || `Walking: ${command}`);
  };

  // --- Filter Logic ---

  const filteredOffsetRows = useMemo(() => {
    let rows = offsetRows;
    if (activeGroup !== "All") {
       const substrings = JOINT_GROUPS[activeGroup];
       rows = rows.filter(r => substrings.some(s => r.joint_name.includes(s)));
    }
    const term = offsetFilter.trim().toLowerCase();
    if (term) {
      rows = rows.filter((row) => row.joint_name.toLowerCase().includes(term));
    }
    return rows;
  }, [offsetRows, offsetFilter, activeGroup]);

  // --- Render ---

  return (
    <div className="flex flex-col h-full overflow-hidden bg-gray-50/50">
      <div className="flex-1 w-full p-4 md:p-6 grid grid-cols-1 xl:grid-cols-12 gap-6 overflow-y-auto xl:overflow-hidden">
        
        {/* LEFT COLUMN: Offset Tuner */}
        <div className="xl:col-span-8 flex flex-col gap-6 h-auto xl:h-full xl:overflow-hidden">
          <SectionCard 
            title="Offset Tuner" 
            icon={Sliders} 
            className="flex-1 min-h-[500px] xl:min-h-0"
            actions={
              <div className="flex items-center gap-2">
                 <button 
                  onClick={loadOffsetRows} 
                  disabled={rosState !== "connected" || offsetLoading}
                  className="p-1.5 text-gray-500 hover:text-undip-blue hover:bg-blue-50 rounded-lg transition-colors"
                  title="Reload"
                >
                  <RefreshCw size={16} className={offsetLoading ? "animate-spin" : ""} />
                </button>
                <button 
                  onClick={applyAllOffsets} 
                  disabled={rosState !== "connected" || offsetRows.length === 0}
                  className="px-3 py-1.5 bg-undip-blue text-white rounded-lg text-xs font-bold hover:bg-opacity-90 flex items-center gap-2 transition-colors disabled:opacity-50"
                >
                  <Save size={14} /> Apply All
                </button>
              </div>
            }
          >
            {/* Toolbar */}
            <div className="flex flex-wrap items-center justify-between gap-4 mb-4">
              <div className="flex items-center gap-2 bg-gray-100 p-1 rounded-lg">
                {["All", "Head", "Arms", "Legs"].map(group => (
                  <button
                    key={group}
                    onClick={() => setActiveGroup(group)}
                    className={`px-3 py-1 text-xs font-bold rounded-md transition-all ${activeGroup === group ? 'bg-white text-undip-blue shadow-sm' : 'text-gray-500 hover:text-gray-700'}`}
                  >
                    {group}
                  </button>
                ))}
              </div>
              <div className="flex items-center gap-2">
                <div className="relative">
                  <Search size={14} className="absolute left-2.5 top-1/2 -translate-y-1/2 text-gray-400" />
                  <input
                    type="text"
                    placeholder="Search joints..."
                    value={offsetFilter}
                    onChange={e => setOffsetFilter(e.target.value)}
                    className="pl-8 pr-3 py-1.5 bg-white border border-gray-200 rounded-lg text-xs font-mono focus:outline-none focus:border-undip-blue focus:ring-2 focus:ring-undip-blue/10 w-40"
                  />
                </div>
                <div className="h-4 w-px bg-gray-200 mx-1"></div>
                <button
                  onClick={() => setOffsetTorqueAll(true)}
                  disabled={rosState !== "connected"}
                  className="text-xs font-bold text-green-600 hover:bg-green-50 px-2 py-1 rounded transition-colors disabled:opacity-50"
                >
                  Torque ON
                </button>
                <button
                  onClick={() => setOffsetTorqueAll(false)}
                  disabled={rosState !== "connected"}
                  className="text-xs font-bold text-red-600 hover:bg-red-50 px-2 py-1 rounded transition-colors disabled:opacity-50"
                >
                  Torque OFF
                </button>
              </div>
            </div>

            {offsetError && (
              <div className="mb-4 p-3 bg-red-50 border border-red-100 rounded-xl flex items-center gap-2 text-red-600 text-xs font-bold">
                <AlertCircle size={16} /> {offsetError}
              </div>
            )}

            {/* Table Header */}
            <div className="grid grid-cols-[1.5fr_0.8fr_0.8fr_0.8fr_0.6fr_0.6fr_0.6fr_0.8fr_0.6fr] gap-2 px-2 py-2 bg-gray-50/80 border-y border-gray-100 text-[10px] font-bold uppercase tracking-wider text-gray-500 sticky top-0 z-10 backdrop-blur-sm">
              <div>Joint</div>
              <div className="text-center">Present</div>
              <div className="text-center">Goal</div>
              <div className="text-center">Offset</div>
              <div className="text-center">P</div>
              <div className="text-center">I</div>
              <div className="text-center">D</div>
              <div className="text-center">Torque</div>
              <div className="text-center">Action</div>
            </div>

            {/* Table Body */}
            <div className="space-y-1 mt-1">
              {filteredOffsetRows.map((row) => (
                <div key={row.joint_name} className="group grid grid-cols-[1.5fr_0.8fr_0.8fr_0.8fr_0.6fr_0.6fr_0.6fr_0.8fr_0.6fr] gap-2 items-center px-2 py-2 rounded-lg hover:bg-gray-50 transition-colors text-xs border border-transparent hover:border-gray-100">
                  <div className="font-mono text-gray-700 font-bold truncate" title={row.joint_name}>{row.joint_name}</div>
                  <div className="font-mono text-gray-500 text-center">{Number(row.present_deg).toFixed(1)}</div>
                  
                  <div className="relative group/input">
                    <input
                      type="number" step="0.1"
                      className="w-full text-center bg-transparent border-b border-gray-200 group-hover/input:border-undip-blue focus:border-undip-blue focus:outline-none transition-colors font-mono text-gray-700"
                      value={row.goal_deg}
                      onChange={e => updateOffsetRow(row.joint_name, "goal_deg", e.target.value)}
                    />
                  </div>
                  
                  <div className="relative group/input">
                    <input
                      type="number" step="0.1"
                      className={`w-full text-center bg-transparent border-b border-gray-200 group-hover/input:border-undip-blue focus:border-undip-blue focus:outline-none transition-colors font-mono font-bold ${Math.abs(row.offset_deg) > 0.01 ? "text-undip-blue" : "text-gray-400"}`}
                      value={row.offset_deg}
                      onChange={e => updateOffsetRow(row.joint_name, "offset_deg", e.target.value)}
                    />
                  </div>

                  <input type="number" step="1" className="w-full text-center bg-transparent border-b border-gray-200 focus:border-undip-blue focus:outline-none font-mono text-gray-500" value={row.p_gain} onChange={e => updateOffsetRow(row.joint_name, "p_gain", e.target.value)} />
                  <input type="number" step="1" className="w-full text-center bg-transparent border-b border-gray-200 focus:border-undip-blue focus:outline-none font-mono text-gray-500" value={row.i_gain} onChange={e => updateOffsetRow(row.joint_name, "i_gain", e.target.value)} />
                  <input type="number" step="1" className="w-full text-center bg-transparent border-b border-gray-200 focus:border-undip-blue focus:outline-none font-mono text-gray-500" value={row.d_gain} onChange={e => updateOffsetRow(row.joint_name, "d_gain", e.target.value)} />

                  <div className="flex justify-center">
                    <button
                      onClick={() => setOffsetTorque(row.joint_name, !row.torque_enable)}
                      disabled={rosState !== "connected"}
                      className={`w-8 h-6 flex items-center justify-center rounded transition-colors ${row.torque_enable ? "bg-green-100 text-green-600 hover:bg-green-200" : "bg-red-100 text-red-600 hover:bg-red-200"}`}
                    >
                      <Zap size={12} fill={row.torque_enable ? "currentColor" : "none"} />
                    </button>
                  </div>

                  <div className="flex justify-center">
                    <button
                      onClick={() => { if (publishOffsetRow(row)) sendStatus(`Sent: ${row.joint_name}`); }}
                      disabled={rosState !== "connected"}
                      className="text-undip-blue hover:bg-blue-50 p-1.5 rounded-md transition-colors disabled:opacity-30"
                      title="Apply Row"
                    >
                      <CheckCircle2 size={16} />
                    </button>
                  </div>
                </div>
              ))}
              
              {filteredOffsetRows.length === 0 && (
                <div className="flex flex-col items-center justify-center py-12 text-gray-400">
                  <Search size={32} className="mb-2 opacity-20" />
                  <p className="text-sm">No joints found matching filter.</p>
                </div>
              )}
            </div>

            {/* Bottom Actions */}
             <div className="mt-4 pt-4 border-t border-gray-100 grid grid-cols-3 gap-3">
               <button
                  className="py-2.5 bg-gray-50 text-gray-700 hover:bg-gray-100 border border-gray-200 rounded-xl text-xs font-bold transition-all disabled:opacity-50"
                  onClick={() => sendOffsetCommand("ini_pose", "Offset init pose")}
                  disabled={rosState !== "connected"}
                >
                  Init Pose
                </button>
                <button
                  className="py-2.5 bg-blue-50 text-blue-700 hover:bg-blue-100 border border-blue-200 rounded-xl text-xs font-bold transition-all disabled:opacity-50 flex items-center justify-center gap-1.5"
                  onClick={downloadOffsetYAML}
                  disabled={offsetRows.length === 0}
                >
                  <Download size={14} /> Download
                </button>
                <button
                  className="py-2.5 bg-accent-yellow text-black hover:bg-yellow-400 rounded-xl text-xs font-bold transition-all disabled:opacity-50 shadow-sm"
                  onClick={() => sendOffsetCommand("save", "Offsets saved")}
                  disabled={rosState !== "connected"}
                >
                  Save to Flash
                </button>
             </div>
          </SectionCard>
        </div>

        {/* RIGHT COLUMN: Walking & Utils */}
        <div className="xl:col-span-4 flex flex-col gap-6 h-auto xl:h-full xl:overflow-y-auto custom-scrollbar xl:pr-2">
          
          {/* Walking Parameters */}
          <SectionCard 
            title="Walking Tuner" 
            icon={Sliders}
            actions={
              <button onClick={loadWalkingParams} className="p-1.5 text-gray-400 hover:text-undip-blue hover:bg-blue-50 rounded transition-colors"><RefreshCw size={14}/></button>
            }
          >
            <div className="space-y-5">
              {WALKING_PARAM_FIELDS.map(field => (
                <div key={field.key} className="space-y-2">
                   <div className="flex justify-between items-end">
                      <label className="text-xs font-bold text-gray-500 uppercase tracking-wide">{field.label}</label>
                      <span className="text-xs font-mono font-bold text-undip-blue">{Number(walkingParams[field.key]).toFixed(4)}</span>
                   </div>
                   <div className="flex gap-2">
                     <NumberInput 
                        value={walkingParams[field.key]} 
                        onChange={e => setWalkingParams({ ...walkingParams, [field.key]: e.target.value })} 
                        step={field.step}
                     />
                     {field.quick.map(q => (
                       <button
                          key={q.label}
                          onClick={() => setWalkingParams(p => ({ ...p, [field.key]: (Number(p[field.key]) + q.delta).toFixed(6) }))}
                          className="px-2 bg-gray-100 hover:bg-gray-200 text-gray-600 rounded-lg text-[10px] font-bold transition-colors whitespace-nowrap"
                       >
                         {q.label}
                       </button>
                     ))}
                   </div>
                </div>
              ))}

              <div className="pt-2 grid grid-cols-2 gap-3">
                 <button onClick={applyWalkingParams} className="py-2.5 border border-gray-200 text-gray-700 hover:bg-gray-50 rounded-xl text-xs font-bold transition-all">
                   Apply Only
                 </button>
                 <button onClick={() => { if(applyWalkingParams()) sendWalkingCommand("start", "Updated & Started"); }} className="py-2.5 bg-undip-blue text-white hover:bg-opacity-90 rounded-xl text-xs font-bold transition-all shadow-sm">
                   Apply & Start
                 </button>
              </div>
            </div>
          </SectionCard>

          {/* Walking Control */}
          <SectionCard title="Locomotion Control" icon={Gamepad2}>
             <div className="space-y-3">
                <button
                  className="w-full py-2.5 bg-gradient-to-r from-undip-blue to-blue-900 text-white rounded-xl text-sm font-bold shadow-md hover:shadow-lg transition-all flex items-center justify-center gap-2"
                  onClick={() => enableModulePubRef.current?.publish(new ROSLIB.Message({ data: "walking_module" }))}
                  disabled={rosState !== "connected"}
                >
                  <Cpu size={16} /> Enable Walking Module
                </button>
                
                <div className="grid grid-cols-2 gap-3">
                  <button
                    className="py-3 bg-green-50 text-green-700 border border-green-100 hover:bg-green-100 rounded-xl text-sm font-bold transition-colors flex items-center justify-center gap-2"
                    onClick={() => sendWalkingCommand("start")}
                    disabled={rosState !== "connected"}
                  >
                    <Play size={16} fill="currentColor" /> Start
                  </button>
                  <button
                    className="py-3 bg-red-50 text-red-700 border border-red-100 hover:bg-red-100 rounded-xl text-sm font-bold transition-colors flex items-center justify-center gap-2"
                    onClick={() => sendWalkingCommand("stop")}
                    disabled={rosState !== "connected"}
                  >
                    <Square size={16} fill="currentColor" /> Stop
                  </button>
                </div>

                <div className="grid grid-cols-2 gap-3">
                  <button onClick={() => sendWalkingCommand("balance on")} className="py-2 bg-gray-50 hover:bg-gray-100 text-gray-600 rounded-xl text-xs font-bold border border-gray-100">
                    Balance ON
                  </button>
                  <button onClick={() => sendWalkingCommand("balance off")} className="py-2 bg-gray-50 hover:bg-gray-100 text-gray-600 rounded-xl text-xs font-bold border border-gray-100">
                    Balance OFF
                  </button>
                </div>
                
                 <button
                  className="w-full py-2 bg-yellow-50 text-yellow-700 border border-yellow-100 hover:bg-yellow-100 rounded-xl text-xs font-bold transition-colors"
                  onClick={() => sendWalkingCommand("save")}
                  disabled={rosState !== "connected"}
                >
                  Save Params to Flash
                </button>
             </div>
          </SectionCard>

          {/* Teleop Viz */}
          <SectionCard title="Input Monitor" icon={Gamepad2}>
            <div className="flex justify-center py-2">
              <GamepadVisualizer joy={joyState} rosConnected={rosState === "connected"} publishJoy={publishJoy} />
            </div>
          </SectionCard>

          {/* Manual Param */}
          <SectionCard title="Manual Parameter" icon={Terminal}>
            <div className="space-y-3">
              <div className="space-y-1">
                 <label className="text-[10px] font-bold text-gray-400 uppercase">Node</label>
                 <input type="text" value={paramNode} onChange={e => setParamNode(e.target.value)} className="w-full px-2 py-1.5 bg-gray-50 border border-gray-200 rounded-lg text-xs font-mono" placeholder="Node Name" />
              </div>
              <div className="space-y-1">
                 <label className="text-[10px] font-bold text-gray-400 uppercase">Parameter</label>
                 <input type="text" value={paramName} onChange={e => setParamName(e.target.value)} className="w-full px-2 py-1.5 bg-gray-50 border border-gray-200 rounded-lg text-xs font-mono" placeholder="Param Name" />
              </div>
              <div className="space-y-1">
                 <label className="text-[10px] font-bold text-gray-400 uppercase">Value (Double)</label>
                 <input type="text" value={paramValue} onChange={e => setParamValue(e.target.value)} className="w-full px-2 py-1.5 bg-gray-50 border border-gray-200 rounded-lg text-xs font-mono" placeholder="Value" />
              </div>
              <button 
                onClick={() => {
                   const service = new ROSLIB.Service({ ros, name: `/${paramNode}/set_parameters`, serviceType: "rcl_interfaces/srv/SetParameters" });
                   service.callService(new ROSLIB.ServiceRequest({ parameters: [{ name: paramName, value: makeParamValue("double", paramValue) }] }), () => sendStatus("Param Sent"));
                }}
                className="w-full py-2 bg-gray-800 text-white hover:bg-gray-900 rounded-xl text-xs font-bold transition-colors"
              >
                Set Parameter
              </button>
            </div>
          </SectionCard>

        </div>
      </div>
    </div>
  );
}