import { useEffect, useState, useMemo, useRef } from "react";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
} from "recharts";
import { Settings, Trash2, Pause, Play, Plus, X, Activity } from "lucide-react";

// Color palette matching the design system
const LINE_COLORS = [
  "#002060", // Undip Blue
  "#F4B400", // Accent Yellow
  "#ef4444", // Red
  "#10b981", // Green
  "#8b5cf6", // Violet
  "#ec4899", // Pink
  "#06b6d4", // Cyan
  "#f97316", // Orange
];

const MAX_DATAPOINTS = 300; // Safety cap

export default function ChartPage({ currentMetrics }) {
  const [dataHistory, setDataHistory] = useState([]);
  const [activeKeys, setActiveKeys] = useState(["battery.voltage"]);
  const [paused, setPaused] = useState(false);
  const [windowSeconds, setWindowSeconds] = useState(10);
  const [yDomain, setYDomain] = useState({ min: "auto", max: "auto" });
  
  // Available keys (flattened from metrics)
  const [availableKeys, setAvailableKeys] = useState([]);

  // Flatten nested objects into dot-notation keys
  const flattenObject = (obj, prefix = "") => {
    return Object.keys(obj).reduce((acc, k) => {
      const pre = prefix.length ? prefix + "." : "";
      if (typeof obj[k] === "object" && obj[k] !== null && !Array.isArray(obj[k])) {
        Object.assign(acc, flattenObject(obj[k], pre + k));
      } else if (typeof obj[k] === "number") {
        acc[pre + k] = obj[k];
      }
      return acc;
    }, {});
  };

  useEffect(() => {
    if (currentMetrics) {
      const flat = flattenObject(currentMetrics);
      const keys = Object.keys(flat).sort();
      if (JSON.stringify(keys) !== JSON.stringify(availableKeys)) {
        setAvailableKeys(keys);
      }
    }
  }, [currentMetrics]);

  useEffect(() => {
    if (paused || !currentMetrics) return;

    const now = Date.now() / 1000;
    const flatMetrics = flattenObject(currentMetrics);
    
    const newDataPoint = {
      timestamp: now,
      timeLabel: new Date().toLocaleTimeString([], { hour12: false, hour: "2-digit", minute: "2-digit", second: "2-digit" }),
      ...flatMetrics,
    };

    setDataHistory((prev) => {
      const cutoffTime = now - windowSeconds;
      const filtered = prev.filter((pt) => pt.timestamp > cutoffTime);
      
      if (filtered.length > MAX_DATAPOINTS) {
        return [...filtered.slice(filtered.length - MAX_DATAPOINTS), newDataPoint];
      }
      return [...filtered, newDataPoint];
    });
  }, [currentMetrics, paused, windowSeconds]);

  const toggleKey = (key) => {
    if (activeKeys.includes(key)) {
      setActiveKeys(activeKeys.filter((k) => k !== key));
    } else {
      setActiveKeys([...activeKeys, key]);
    }
  };

  return (
    <div className="flex flex-col lg:grid lg:grid-cols-[1fr_300px] gap-6 h-full p-4 md:p-8 overflow-y-auto lg:overflow-hidden">
      {/* LEFT: Chart Area */}
      <div className="flex flex-col gap-6 h-full min-h-[400px]">
        <div className="bg-white p-4 rounded-2xl border border-gray-200 shadow-sm flex-1 min-h-0 flex flex-col">
          <div className="flex justify-between items-center mb-4">
            <h2 className="text-lg font-bold font-display text-gray-800">Live Data Stream</h2>
            <div className="flex gap-2">
              <button
                onClick={() => setDataHistory([])}
                className="p-2 hover:bg-red-50 text-red-600 rounded-lg transition-colors"
                title="Clear Data"
              >
                <Trash2 size={18} />
              </button>
              <button
                onClick={() => setPaused(!paused)}
                className={`p-2 rounded-lg transition-colors flex items-center gap-2 font-bold text-sm ${
                  paused ? "bg-accent-yellow text-black" : "bg-gray-100 text-gray-700"
                }`}
              >
                {paused ? <Play size={18} /> : <Pause size={18} />}
                {paused ? "RESUME" : "PAUSE"}
              </button>
            </div>
          </div>

          <div className="flex-1 w-full min-h-0">
            {activeKeys.length === 0 ? (
              <div className="h-full flex flex-col items-center justify-center text-gray-400">
                <Activity size={48} className="mb-4 opacity-20" />
                <p>Select data points from the panel to visualize.</p>
              </div>
            ) : (
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={dataHistory}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
                  <XAxis 
                    dataKey="timeLabel" 
                    tick={{ fontSize: 10, fill: "#64748b" }} 
                    interval="preserveStartEnd"
                  />
                  <YAxis 
                    domain={[yDomain.min, yDomain.max]} 
                    tick={{ fontSize: 10, fill: "#64748b" }} 
                    width={40}
                  />
                  <Tooltip 
                    contentStyle={{ borderRadius: "12px", border: "1px solid #e2e8f0", boxShadow: "0 4px 6px -1px rgb(0 0 0 / 0.1)" }}
                    itemStyle={{ fontSize: "12px", padding: 0 }}
                    labelStyle={{ fontSize: "10px", color: "#94a3b8", marginBottom: "4px" }}
                  />
                  <Legend />
                  {activeKeys.map((key, index) => (
                    <Line
                      key={key}
                      type="monotone"
                      dataKey={key}
                      stroke={LINE_COLORS[index % LINE_COLORS.length]}
                      dot={false}
                      strokeWidth={2}
                      isAnimationActive={false} 
                    />
                  ))}
                </LineChart>
              </ResponsiveContainer>
            )}
          </div>
        </div>

        {/* Bottom Configuration Bar */}
        <div className="bg-white p-4 rounded-2xl border border-gray-200 shadow-sm flex gap-4 md:gap-6 items-center flex-wrap">
          <div className="flex items-center gap-2">
            <Settings size={16} className="text-gray-400" />
            <span className="text-xs font-bold text-gray-500 uppercase tracking-wider">Config</span>
          </div>
          
          <div className="h-8 w-px bg-gray-200 hidden md:block"></div>

          <div className="flex items-center gap-2 md:gap-3 flex-1 min-w-[120px]">
            <label className="text-xs font-medium text-gray-600 whitespace-nowrap">Window (s)</label>
            <input
              type="number"
              value={windowSeconds}
              onChange={(e) => setWindowSeconds(Number(e.target.value))}
              className="w-full md:w-16 px-2 py-1 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:border-undip-blue outline-none"
            />
          </div>

          <div className="flex items-center gap-2 md:gap-3 flex-1 min-w-[100px]">
            <label className="text-xs font-medium text-gray-600">Y-Min</label>
            <input
              type="text"
              placeholder="auto"
              value={yDomain.min}
              onChange={(e) => setYDomain({ ...yDomain, min: e.target.value === "auto" || e.target.value === "" ? "auto" : Number(e.target.value) })}
              className="w-full md:w-16 px-2 py-1 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:border-undip-blue outline-none"
            />
          </div>

          <div className="flex items-center gap-2 md:gap-3 flex-1 min-w-[100px]">
            <label className="text-xs font-medium text-gray-600">Y-Max</label>
            <input
              type="text"
              placeholder="auto"
              value={yDomain.max}
              onChange={(e) => setYDomain({ ...yDomain, max: e.target.value === "auto" || e.target.value === "" ? "auto" : Number(e.target.value) })}
              className="w-full md:w-16 px-2 py-1 bg-gray-50 border border-gray-200 rounded-lg text-sm font-mono focus:border-undip-blue outline-none"
            />
          </div>
        </div>
      </div>

      {/* RIGHT: Data Select Panel */}
      <div className="flex flex-col h-full overflow-hidden min-h-[300px]">
        <div className="bg-white rounded-2xl border border-gray-200 shadow-sm flex flex-col h-full overflow-hidden">
          <div className="p-4 border-b border-gray-100">
            <h2 className="text-sm font-bold font-display text-gray-800">Data Sources</h2>
            <p className="text-xs text-gray-500 mt-1">Select streams to plot</p>
          </div>
          
          <div className="flex-1 overflow-y-auto p-2 custom-scrollbar">
            <div className="flex flex-col gap-1">
              {availableKeys.map((key) => {
                const isActive = activeKeys.includes(key);
                const colorIndex = activeKeys.indexOf(key);
                const color = isActive ? LINE_COLORS[colorIndex % LINE_COLORS.length] : "#94a3b8";

                return (
                  <button
                    key={key}
                    onClick={() => toggleKey(key)}
                    className={`flex items-center justify-between p-2 rounded-lg text-left transition-all ${
                      isActive 
                        ? "bg-gray-50 border border-gray-200" 
                        : "hover:bg-gray-50 border border-transparent"
                    }`}
                  >
                    <div className="flex items-center gap-3 overflow-hidden">
                      <div 
                        className="w-3 h-3 rounded-full flex-shrink-0 transition-colors"
                        style={{ backgroundColor: color }}
                      ></div>
                      <span className={`text-xs font-mono truncate ${isActive ? "text-gray-900 font-bold" : "text-gray-500"}`}>
                        {key}
                      </span>
                    </div>
                    {isActive && <X size={12} className="text-gray-400" />}
                  </button>
                );
              })}
              
              {availableKeys.length === 0 && (
                <div className="p-4 text-center text-xs text-gray-400">
                  No metrics received yet. Ensure ROS bridge is connected.
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
