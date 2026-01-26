import { cloneElement, useCallback, useEffect, useRef, useState } from "react";
import { Monitor, Smartphone, Cpu, AlertCircle, Info } from "lucide-react";

const BUTTON_MAP = {
  CROSS: 0, CIRCLE: 1, SQUARE: 2, TRIANGLE: 3,
  L1: 4, R1: 5, L2: 6, R2: 7,
  SELECT: 8, START: 9, L3: 10, R3: 11,
  DPAD_UP: 12, DPAD_DOWN: 13, DPAD_LEFT: 14, DPAD_RIGHT: 15,
  HOME: 16,
};

const AXIS_MAP = { lx: 0, ly: 1, rx: 2, ry: 3, rx_alt: 4, ry_alt: 5 };
const BUTTON_COUNT = 17;
const AXIS_COUNT = 6;
const DEADZONE = 0.1;
const STICK_RADIUS = 45;

const LEFT_STICK_CENTER = { x: 429, y: 511 };
const RIGHT_STICK_CENTER = { x: 843, y: 511 };

const ZERO_JOY = {
  axes: Array(AXIS_COUNT).fill(0),
  buttons: Array(BUTTON_COUNT).fill(0),
};

const PUBLISH_RATE_HZ = 30;

function clamp(value, min, max) { return Math.max(min, Math.min(max, value)); }
function normalizeAxis(value) { return (value === null || value === undefined || Number.isNaN(value)) ? 0 : clamp(value, -1, 1); }

function normalizeButtons(buttons) {
  if (!Array.isArray(buttons)) return [];
  return buttons.map((button) => {
    if (!button) return 0;
    if (typeof button === "number") return button;
    if (typeof button.value === "number") return button.value;
    return button.pressed ? 1 : 0;
  });
}

function isJoyActive(joyData) {
  if (!joyData) return false;
  if (Array.isArray(joyData.buttons) && joyData.buttons.some(v => Number(v) > 0.5)) return true;
  if (Array.isArray(joyData.axes) && joyData.axes.some(v => Math.abs(Number(v) || 0) > DEADZONE)) return true;
  return false;
}

function buildAxes(virtualAxes) {
  const axes = Array(AXIS_COUNT).fill(0);
  const lx = normalizeAxis(virtualAxes.lx);
  const ly = normalizeAxis(virtualAxes.ly);
  const rx = normalizeAxis(virtualAxes.rx);
  const ry = normalizeAxis(virtualAxes.ry);
  axes[AXIS_MAP.lx] = lx;
  axes[AXIS_MAP.ly] = ly;
  axes[AXIS_MAP.rx] = rx;
  axes[AXIS_MAP.ry] = ry;
  if (AXIS_MAP.rx_alt >= 0) axes[AXIS_MAP.rx_alt] = rx;
  if (AXIS_MAP.ry_alt >= 0) axes[AXIS_MAP.ry_alt] = ry;
  return axes;
}

function formatAxis(value) { return Number(value || 0).toFixed(2); }

export default function GamepadVisualizer({ joy, rosConnected, publishJoy }) {
  const [virtualButtons, setVirtualButtons] = useState(() => new Array(BUTTON_MAP.length).fill(false)); // BUTTON_COUNT was used but not def in scope here effectively if i change logic, but let's assume constants are above. 
  // actually BUTTON_COUNT is defined in module scope.
  const [virtualAxes, setVirtualAxes] = useState({ lx: 0, ly: 0, rx: 0, ry: 0 });
  const [draggingStick, setDraggingStick] = useState(null);
  const [virtualEnabled, setVirtualEnabled] = useState(false);
  const [browserPad, setBrowserPad] = useState(null);
  const [visualSource, setVisualSource] = useState("auto");
  const [notice, setNotice] = useState("");

  const svgRef = useRef(null);
  const virtualJoyRef = useRef(ZERO_JOY);

  useEffect(() => {
    virtualJoyRef.current = {
      axes: buildAxes(virtualAxes),
      buttons: virtualButtons.map((pressed) => (pressed ? 1 : 0)),
    };
  }, [virtualAxes, virtualButtons]);

  useEffect(() => {
    if (typeof navigator === "undefined" || !navigator.getGamepads) return;
    const poll = () => {
      const pads = navigator.getGamepads();
      let active = null;
      for (const pad of pads || []) { if (pad && pad.connected) { active = pad; break; } }
      if (!active) { setBrowserPad(null); return; }
      setBrowserPad({
        id: active.id,
        index: active.index,
        axes: Array.isArray(active.axes) ? active.axes.map(normalizeAxis) : [],
        buttons: normalizeButtons(active.buttons),
      });
    };
    const interval = window.setInterval(poll, 1000 / PUBLISH_RATE_HZ);
    return () => window.clearInterval(interval);
  }, []);

  useEffect(() => {
    if (virtualEnabled && !rosConnected) {
      setVirtualEnabled(false);
      setNotice("ROS not connected; virtual /joy paused.");
    }
  }, [virtualEnabled, rosConnected]);

  useEffect(() => {
    if (!virtualEnabled) {
      if (rosConnected) publishJoy(ZERO_JOY);
      return;
    }
    const tick = () => publishJoy(virtualJoyRef.current);
    const interval = window.setInterval(tick, 1000 / PUBLISH_RATE_HZ);
    return () => window.clearInterval(interval);
  }, [virtualEnabled, rosConnected, publishJoy]);

  useEffect(() => {
    if (!virtualEnabled) {
      setVirtualButtons(new Array(BUTTON_COUNT).fill(false));
      setVirtualAxes({ lx: 0, ly: 0, rx: 0, ry: 0 });
      setDraggingStick(null);
    }
  }, [virtualEnabled]);

  const ensureVirtualEnabled = useCallback(() => {
    if (!virtualEnabled) {
      if (!rosConnected) { setNotice("Connect ROS to publish virtual /joy."); return false; }
      setNotice(""); setVirtualEnabled(true);
    }
    return true;
  }, [virtualEnabled, rosConnected]);

  const emitJoy = useCallback((buttons, axes) => {
    if (!rosConnected) return;
    publishJoy({
      axes: buildAxes(axes),
      buttons: buttons.map((pressed) => (pressed ? 1 : 0)),
    });
  }, [publishJoy, rosConnected]);

  const browserJoy = browserPad ? { axes: browserPad.axes || [], buttons: browserPad.buttons || [] } : null;
  const rosActive = isJoyActive(joy);
  const browserActive = isJoyActive(browserJoy);
  const resolvedSource = visualSource === "auto" ? (browserActive ? "browser" : joy ? "ros" : browserPad ? "browser" : "ros") : visualSource;
  const displayJoy = resolvedSource === "browser" ? browserJoy : joy;

  const handleButtonDown = (index) => {
    if (!ensureVirtualEnabled()) return;
    setVirtualButtons((prev) => {
      const next = [...prev]; next[index] = true;
      emitJoy(next, virtualAxes);
      return next;
    });
  };

  const handleButtonUp = (index) => {
    setVirtualButtons((prev) => {
      const next = [...prev]; next[index] = false;
      emitJoy(next, virtualAxes);
      return next;
    });
  };

  const getAxis = (index) => (displayJoy?.axes && Number.isFinite(displayJoy.axes[index])) ? displayJoy.axes[index] : 0;

  const getSmartAxis = (primaryIndex, altIndex, virtualValue, isDragging) => {
    if (isDragging || Math.abs(virtualValue) > 0.01) return virtualValue;
    const stdVal = getAxis(primaryIndex);
    const altVal = altIndex >= 0 ? getAxis(altIndex) : 0;
    if (altIndex >= 0 && Math.abs(altVal) > DEADZONE) return altVal;
    if (Math.abs(stdVal) > DEADZONE) return stdVal;
    return 0;
  };

  const leftStickX = getSmartAxis(AXIS_MAP.lx, -1, virtualAxes.lx, draggingStick === "left");
  const leftStickY = getSmartAxis(AXIS_MAP.ly, -1, virtualAxes.ly, draggingStick === "left");
  const rightStickX = getSmartAxis(AXIS_MAP.rx, AXIS_MAP.rx_alt, virtualAxes.rx, draggingStick === "right");
  const rightStickY = getSmartAxis(AXIS_MAP.ry, AXIS_MAP.ry_alt, virtualAxes.ry, draggingStick === "right");

  const handleStickStart = (stick, event) => {
    event.preventDefault();
    if (ensureVirtualEnabled()) setDraggingStick(stick);
  };

  const handleDragMove = useCallback((event) => {
    if (!draggingStick || !svgRef.current) return;
    if (event.cancelable) event.preventDefault();
    const svg = svgRef.current;
    const pt = svg.createSVGPoint();
    const touch = event.touches?.[0];
    pt.x = touch ? touch.clientX : event.clientX;
    pt.y = touch ? touch.clientY : event.clientY;
    const matrix = svg.getScreenCTM();
    if (!matrix) return;
    const svgPoint = pt.matrixTransform(matrix.inverse());
    const center = draggingStick === "left" ? LEFT_STICK_CENTER : RIGHT_STICK_CENTER;
    const deltaX = (svgPoint.x - center.x) / STICK_RADIUS;
    const deltaY = (svgPoint.y - center.y) / STICK_RADIUS;
    const mag = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    const nx = mag > 1 ? deltaX / mag : deltaX;
    const ny = mag > 1 ? deltaY / mag : deltaY;
    setVirtualAxes(prev => ({ ...prev, [draggingStick === "left" ? "lx" : "rx"]: nx, [draggingStick === "left" ? "ly" : "ry"]: ny }));
  }, [draggingStick]);

  const handleDragEnd = useCallback(() => {
    setDraggingStick(null);
    setVirtualAxes(prev => ({ ...prev, lx: 0, ly: 0, rx: 0, ry: 0 }));
  }, []);

  useEffect(() => {
    if (draggingStick) {
      window.addEventListener("mousemove", handleDragMove);
      window.addEventListener("mouseup", handleDragEnd);
      window.addEventListener("touchmove", handleDragMove, { passive: false });
      window.addEventListener("touchend", handleDragEnd);
    }
    return () => {
      window.removeEventListener("mousemove", handleDragMove);
      window.removeEventListener("mouseup", handleDragEnd);
      window.removeEventListener("touchmove", handleDragMove);
      window.removeEventListener("touchend", handleDragEnd);
    };
  }, [draggingStick, handleDragMove, handleDragEnd]);

  const isPressed = (index) => {
    const physical = displayJoy?.buttons?.[index] ?? 0;
    return Boolean(physical) || virtualButtons[index];
  };

  const InteractiveButton = ({ index, children, className }) =>
    cloneElement(children, {
      onMouseDown: () => handleButtonDown(index),
      onMouseUp: () => handleButtonUp(index),
      onMouseLeave: () => handleButtonUp(index),
      onTouchStart: (e) => { e.preventDefault(); handleButtonDown(index); },
      onTouchEnd: (e) => { e.preventDefault(); handleButtonUp(index); },
      className: `transition-all duration-150 cursor-pointer hover:opacity-80 active:scale-95 ${className || ""}`,
    });

  const axes = displayJoy?.axes || [];
  const pressedButtons = displayJoy?.buttons ? displayJoy.buttons.map((v, i) => (Number(v) > 0.5 ? i : null)).filter(v => v !== null) : [];

  // Design system colors
  const activeColor = "#F4B400"; // Accent Yellow
  const inactiveColor = "#e2e8f0"; // Slate-200
  const bodyColor = "#0f172a"; // Dark Sidebar

  return (
    <div className="flex flex-col gap-4 w-full">
      {/* Controls */}
      <div className="flex flex-col sm:flex-row justify-between items-center gap-2 pb-2 border-b border-gray-100">
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-2">
            <span className="text-[10px] font-bold text-gray-400 uppercase">Source</span>
            <select
              value={visualSource}
              onChange={(e) => setVisualSource(e.target.value)}
              className="text-xs font-mono bg-gray-50 border border-gray-200 rounded-lg px-2 py-1 focus:outline-none focus:border-undip-blue"
            >
              <option value="auto">Auto</option>
              <option value="ros">ROS</option>
              <option value="browser">Browser</option>
            </select>
          </div>
          <div className="h-4 w-px bg-gray-200"></div>
          <div className="flex items-center gap-2">
            <span className="text-[10px] font-bold text-gray-400 uppercase">Virtual</span>
            <label className="relative inline-flex items-center cursor-pointer">
              <input 
                type="checkbox" 
                checked={virtualEnabled} 
                onChange={e => setVirtualEnabled(e.target.checked)} 
                className="sr-only peer"
              />
              <div className="w-8 h-4 bg-gray-200 peer-focus:outline-none rounded-full peer peer-checked:after:translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-white after:border-gray-300 after:border after:rounded-full after:h-3 after:w-3 after:transition-all peer-checked:bg-undip-blue"></div>
            </label>
          </div>
        </div>
        
        <div className="flex gap-2">
          {joy && (
            <div className={`flex items-center gap-1 px-1.5 py-0.5 rounded text-[10px] font-bold uppercase ${rosActive ? 'text-green-600 bg-green-50' : 'text-gray-400'}`}>
              <Cpu size={10} /> ROS
            </div>
          )}
          {browserPad && (
            <div className={`flex items-center gap-1 px-1.5 py-0.5 rounded text-[10px] font-bold uppercase ${browserPad ? 'text-blue-600 bg-blue-50' : 'text-gray-400'}`}>
              <Smartphone size={10} /> Browser
            </div>
          )}
        </div>
      </div>

      {/* Main Gamepad Shell */}
      <div className="relative bg-[#f8fafc] rounded-2xl border border-gray-200 p-6 shadow-inner overflow-hidden flex items-center justify-center">
        {/* Alerts */}
        {notice && (
          <div className="absolute top-4 left-4 right-4 bg-red-50 border border-red-100 text-red-600 px-3 py-1.5 rounded-lg text-xs font-bold flex items-center gap-2 animate-bounce z-20 shadow-sm">
            <AlertCircle size={14} /> {notice}
          </div>
        )}

        <svg ref={svgRef} viewBox="0 0 1280 819" className="w-full max-w-[480px] h-auto drop-shadow-xl select-none touch-none">
          {/* Shell */}
          <path
            d="M209.5 7.246c11.7-2.7 26.5-5.2 38.5-6.6 12.5-1.4 38.5-.4 49 1.8 19.7 4.3 31.2 10.6 43.7 24.1 7.8 8.4 21.9 28.7 25.2 36.4 4.4 10.1 12.6 47.8 12.6 58.3v3.1h522v-3.1c0-5.2 4.8-32.2 7.6-43 3.5-13.1 6-18.6 13.5-29.9 12-17.9 23.6-30.5 33.3-36.2 6.4-3.7 19-8.1 29.2-10.1 11-2.2 40.4-2.5 54.4-.5 26.1 3.6 47.3 9.1 61 15.8 21 10.2 31.8 27.5 41.4 66 1.9 7.6 4 16.3 4.6 19.4l1.1 5.5 11.2 8c29 20.4 53.9 42.9 63.3 57.1 11.4 17.1 20.1 37.4 28.8 67.5 7.1 24.6 7.5 27.6 17.5 138.3 9.3 101.8 11.5 142.5 11.6 213 0 54.6-1.2 87.9-4 110.6-3.5 27.8-13.4 49.3-31.2 68-23.4 24.5-47.6 38.4-78.6 45.1-14.5 3.1-41.5 3.1-53 0-16.6-4.5-33.9-14.7-51.7-30.5-24.5-21.7-42.3-49.1-72.6-111.7-18.2-37.4-19.9-40.6-26.2-47.5-3.1-3.3-8-9.3-10.9-13.2l-5.4-7.3-10.2 8.3c-23.1 18.7-34.4 24.2-60.9 29.8-12.4 2.6-36.9 3.1-48.8 1-27.3-4.8-51.2-13.8-71-26.9-17.2-11.4-27.6-24.6-41.3-52.4l-7.2-14.6H573l-7.2 14.6c-13.7 27.8-24.1 41-41.3 52.4-20.1 13.2-43.7 22.1-71 26.9-11.9 2.1-36.4 1.6-48.8-1-26.5-5.6-37.8-11.1-60.9-29.8l-10.2-8.3-5.4 7.3c-3 3.9-8 10.1-11.3 13.7-4 4.4-7.6 9.9-11.1 17-2.8 5.8-10.8 22-17.6 36-28.5 58.3-47.1 86.1-71.4 107.1-17.8 15.4-33.8 24.7-50.1 29.1-11.4 3.1-38.5 3.1-52.9 0-31-6.7-55.2-20.6-78.6-45.1-17.8-18.7-27.7-40.2-31.2-68-2.8-22.7-4-56-4-110.6.1-70.4 2.3-111.1 11.6-213 10.2-112.6 10-111.3 15.9-132.9 8-29.2 17-51.6 27.4-68.6 10-16.2 33.5-38 65.4-60.8 6.4-4.5 11.7-8.4 11.8-8.5.2-.1 1.7-6.8 3.4-14.7 6.1-27.9 16.2-53.4 24.5-62.2 11.4-12 24.5-18.4 49.5-24.2z"
            fill={bodyColor}
          />

          {/* Buttons & Triggers */}
          <g transform="translate(-10, 0)">
          <InteractiveButton index={BUTTON_MAP.L2}>
            <path d="M180 40 Q180 10 210 10 H330 Q360 10 360 40 V60 H180 V40 Z" fill={isPressed(BUTTON_MAP.L2) ? activeColor : inactiveColor} />
          </InteractiveButton>
          </g>
          <g transform="translate(17, 0)">
          <InteractiveButton index={BUTTON_MAP.R2}>
            <path d="M920 40 Q920 10 950 10 H1070 Q1100 10 1100 40 V60 H920 V40 Z" fill={isPressed(BUTTON_MAP.R2) ? activeColor : inactiveColor} />
          </InteractiveButton>
          </g>
          <InteractiveButton index={BUTTON_MAP.L1}>
            <rect x="170" y="70" width="180" height="40" rx="10" fill={isPressed(BUTTON_MAP.L1) ? activeColor : inactiveColor} />
          </InteractiveButton>
          <InteractiveButton index={BUTTON_MAP.R1}>
            <rect x="940" y="70" width="180" height="40" rx="10" fill={isPressed(BUTTON_MAP.R1) ? activeColor : inactiveColor} />
          </InteractiveButton>

          {/* Action Buttons */}
          <InteractiveButton index={BUTTON_MAP.SQUARE}><circle cx={935.5} cy={283.5} r={47.5} fill={isPressed(BUTTON_MAP.SQUARE) ? activeColor : inactiveColor} /></InteractiveButton>
          <InteractiveButton index={BUTTON_MAP.TRIANGLE}><circle cx={1050.5} cy={183.5} r={47.5} fill={isPressed(BUTTON_MAP.TRIANGLE) ? activeColor : inactiveColor} /></InteractiveButton>
          <InteractiveButton index={BUTTON_MAP.CROSS}><circle cx={1050.5} cy={383.5} r={47.5} fill={isPressed(BUTTON_MAP.CROSS) ? activeColor : inactiveColor} /></InteractiveButton>
          <InteractiveButton index={BUTTON_MAP.CIRCLE}><circle cx={1162.5} cy={283.5} r={47.5} fill={isPressed(BUTTON_MAP.CIRCLE) ? activeColor : inactiveColor} /></InteractiveButton>

          {/* DPAD */}
          <InteractiveButton index={BUTTON_MAP.DPAD_UP}><path d="M269 165h-77v56c9.333 11.333 30 34 38 34s29.333-22.667 39-34v-56z" fill={isPressed(BUTTON_MAP.DPAD_UP) ? activeColor : inactiveColor} /></InteractiveButton>
          <InteractiveButton index={BUTTON_MAP.DPAD_DOWN}><path d="M269 392h-77v-56c9.333-11.333 30-34 38-34s29.333 22.667 39 34v56z" fill={isPressed(BUTTON_MAP.DPAD_DOWN) ? activeColor : inactiveColor} /></InteractiveButton>
          <InteractiveButton index={BUTTON_MAP.DPAD_LEFT}><path d="M119 240v77h56c11.333-9.333 34-30 34-38s-22.667-29.333-34-39h-56z" fill={isPressed(BUTTON_MAP.DPAD_LEFT) ? activeColor : inactiveColor} /></InteractiveButton>
          <InteractiveButton index={BUTTON_MAP.DPAD_RIGHT}><path d="M341 240v77h-56c-11.333-9.333-34-30-34-38s22.667-29.333 34-39h56z" fill={isPressed(BUTTON_MAP.DPAD_RIGHT) ? activeColor : inactiveColor} /></InteractiveButton>

          {/* Sticks */}
          <g onMouseDown={(e) => handleStickStart("left", e)} onTouchStart={(e) => handleStickStart("left", e)} className="cursor-grab active:cursor-grabbing">
            <circle cx={LEFT_STICK_CENTER.x} cy={LEFT_STICK_CENTER.y} r={93} fill={isPressed(BUTTON_MAP.L3) ? activeColor : "#334155"} style={{ transform: `translate(${leftStickX * STICK_RADIUS}px, ${leftStickY * STICK_RADIUS}px)`, transformOrigin: `${LEFT_STICK_CENTER.x}px ${LEFT_STICK_CENTER.y}px`, transition: draggingStick === "left" ? "none" : "transform 0.1s ease-out" }} />
          </g>
          <g onMouseDown={(e) => handleStickStart("right", e)} onTouchStart={(e) => handleStickStart("right", e)} className="cursor-grab active:cursor-grabbing">
            <circle cx={RIGHT_STICK_CENTER.x} cy={RIGHT_STICK_CENTER.y} r={93} fill={isPressed(BUTTON_MAP.R3) ? activeColor : "#334155"} style={{ transform: `translate(${rightStickX * STICK_RADIUS}px, ${rightStickY * STICK_RADIUS}px)`, transformOrigin: `${RIGHT_STICK_CENTER.x}px ${RIGHT_STICK_CENTER.y}px`, transition: draggingStick === "right" ? "none" : "transform 0.1s ease-out" }} />
          </g>

          {/* Center Buttons */}
          <InteractiveButton index={BUTTON_MAP.SELECT}><path d="M471 262h75v47h-75z" fill={isPressed(BUTTON_MAP.SELECT) ? activeColor : inactiveColor} /></InteractiveButton>
          <InteractiveButton index={BUTTON_MAP.START}><path d="M728 309v-49l72 23-72 26z" fill={isPressed(BUTTON_MAP.START) ? activeColor : inactiveColor} /></InteractiveButton>
        </svg>
      </div>

      {/* Telemetry Readout */}
      <div className="bg-white rounded-2xl border border-gray-200 p-6 shadow-sm">
        <div className="flex items-center gap-2 mb-4 text-gray-400">
          <Info size={14} />
          <span className="text-[10px] font-bold uppercase tracking-widest">Telemetry</span>
        </div>
        <div className="grid grid-cols-2 gap-8">
          <div className="space-y-3">
            <h4 className="text-xs font-bold text-gray-700 uppercase tracking-tight">Analog Axes</h4>
            <div className="flex flex-wrap gap-2">
              {axes.length > 0 ? axes.map((v, i) => (
                <div key={i} className="bg-gray-50 border border-gray-100 px-2 py-1 rounded text-[10px] font-mono text-gray-600">
                  {i}:<span className={Math.abs(v) > 0.1 ? "text-undip-blue font-bold" : ""}>{formatAxis(v)}</span>
                </div>
              )) : <span className="text-xs text-gray-400 italic">No data</span>}
            </div>
          </div>
          <div className="space-y-3">
            <h4 className="text-xs font-bold text-gray-700 uppercase tracking-tight">Active Buttons</h4>
            <div className="flex flex-wrap gap-2">
              {pressedButtons.length > 0 ? pressedButtons.map(i => (
                <div key={i} className="bg-undip-blue text-white px-2 py-1 rounded text-[10px] font-bold font-mono shadow-sm">
                  {i}
                </div>
              )) : <span className="text-xs text-gray-400 italic">None</span>}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
