import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { toast } from "react-toastify";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  Tooltip,
  CartesianGrid,
  ResponsiveContainer,
  Legend,
} from "recharts";
import "./dashboard.css";
import { telemetryApi, robotApi } from "../../services/api";
import { useAuth } from "../../context/AuthContext";
import ManualControl from "./ManualControl";

const NOVNC_URL =
  import.meta.env.VITE_NOVNC_URL ||
  "http://192.168.0.165:6080/vnc.html?autoconnect=true&resize=scale";

const CHART_DARK = {
  grid: { stroke: "rgba(255,255,255,0.06)", strokeDasharray: "3 3" },
  tick: { fill: "#475569", fontSize: 11 },
  tooltip: {
    contentStyle: {
      background: "rgba(13,17,23,0.95)",
      border: "1px solid rgba(255,255,255,0.07)",
      borderRadius: "10px",
      color: "#e2e8f0",
    },
    labelStyle: { color: "#94a3b8" },
    itemStyle: { color: "#e2e8f0" },
  },
};

function tempColor(t) {
  if (t < 50) return "#39ff14";
  if (t < 70) return "#ffaa00";
  return "#ff006e";
}

function usageColor(u) {
  if (u < 60) return "#00d4ff";
  if (u < 85) return "#ffaa00";
  return "#ff006e";
}

function throttleInfo(val) {
  if (val === undefined || val === null || val < 0)
    return { label: "Unknown", sub: "Cannot read power state", color: "#475569", symbol: "?" };
  if (val & 0x1)
    return { label: "Under-voltage!", sub: "Check your power supply", color: "#ff006e", symbol: "✕" };
  if (val & 0x4)
    return { label: "Throttled!", sub: "CPU frequency reduced", color: "#ff006e", symbol: "✕" };
  if (val & 0x10000)
    return { label: "UV Occurred", sub: "Under-voltage since boot", color: "#ffaa00", symbol: "⚠" };
  if (val & 0x40000)
    return { label: "Throttle Occurred", sub: "Throttling since boot", color: "#ffaa00", symbol: "⚠" };
  return { label: "Power OK", sub: "Supply is stable", color: "#39ff14", symbol: "✓" };
}

// SVG arc gauge helper — draws a half-circle arc gauge
function ArcGauge({ value, max = 100, color, size = 120 }) {
  const r = 50;
  const cx = 60;
  const cy = 60;
  const circ = Math.PI * r;
  const pct = Math.min(value / max, 1);
  const offset = circ * (1 - pct);

  return (
    <svg viewBox="0 0 120 68" className="metric-arc-svg" style={{ width: size, height: size * 0.57 }}>
      <path
        d={`M ${cx - r} ${cy} A ${r} ${r} 0 0 1 ${cx + r} ${cy}`}
        fill="none"
        stroke="rgba(255,255,255,0.07)"
        strokeWidth="10"
        strokeLinecap="round"
      />
      <path
        d={`M ${cx - r} ${cy} A ${r} ${r} 0 0 1 ${cx + r} ${cy}`}
        fill="none"
        stroke={color}
        strokeWidth="10"
        strokeLinecap="round"
        strokeDasharray={`${circ}`}
        strokeDashoffset={`${offset}`}
        style={{ transition: "stroke-dashoffset 0.6s cubic-bezier(0.4,0,0.2,1)", filter: `drop-shadow(0 0 5px ${color})` }}
      />
    </svg>
  );
}

// SVG donut for bins
function DonutGauge({ value, color }) {
  const r = 62;
  const circ = 2 * Math.PI * r;
  const offset = circ * (1 - Math.min(value / 100, 1));

  return (
    <svg className="donut-svg" viewBox="0 0 160 160">
      <circle className="donut-track" cx="80" cy="80" r={r} />
      <circle
        className="donut-ring"
        cx="80" cy="80" r={r}
        stroke={color}
        strokeDasharray={`${circ}`}
        strokeDashoffset={`${offset}`}
      />
    </svg>
  );
}

export default function Dashboard() {
  const { user, logout } = useAuth();
  const navigate = useNavigate();
  const [last, setLast] = useState(null);
  const [history, setHistory] = useState([]);
  const [activeTab, setActiveTab] = useState("overview");
  const [isBinEmpty, setIsBinEmpty] = useState(false);
  const [apiError, setApiError] = useState(false);
  const [shuttingDown, setShuttingDown] = useState(false);

  // Fetch history on mount
  useEffect(() => {
    async function loadHistory() {
      try {
        const data = await telemetryApi.getHistory(30);
        const mapped = data.reverse().map(mapTelemetry);
        setHistory(mapped);
        if (mapped.length > 0) setLast(mapped[mapped.length - 1]);
      } catch (err) {
        if (err.message !== "No telemetry found") {
          console.error("Failed to load history:", err.message);
        }
      }
    }
    loadHistory();
  }, []);

  // Poll latest telemetry every 3 seconds
  useEffect(() => {
    const poll = async () => {
      try {
        const res = await telemetryApi.getLatest();
        const sample = mapTelemetry(res.data);
        setLast(sample);
        setHistory((prev) => [...prev, sample].slice(-30));
        setApiError(false);
      } catch (err) {
        console.error("Telemetry poll failed:", err.message);
        setApiError(true);
      }
    };

    const id = setInterval(poll, 3000);
    poll();
    return () => clearInterval(id);
  }, []);

  function mapTelemetry(t) {
    return {
      speed: Number((t.speed || 0).toFixed(2)),
      cpuTemp: Math.round(t.cpuTemp || 0),
      cpuUsage: Math.round(t.cpuUsage || 0),
      ramUsage: Math.round(t.ramUsage || 0),
      fanSpeed: Math.round(t.fanSpeed || 0),
      binOrganic: Math.round(t.binOrganic || 0),
      binNonOrganic: Math.round(t.binNonOrganic || 0),
      throttled: t.throttled ?? -1,
      timestamp: t.timestamp ? new Date(t.timestamp).getTime() : Date.now(),
      timeLabel: t.timestamp
        ? new Date(t.timestamp).toLocaleTimeString()
        : new Date().toLocaleTimeString(),
    };
  }

  async function handleEmptyBin() {
    if (isBinEmpty) return;
    setIsBinEmpty(true);
    try {
      await robotApi.emptyBins();
      setLast((prev) => prev ? { ...prev, binOrganic: 0, binNonOrganic: 0 } : null);
      setHistory((prev) =>
        prev.map((item, idx) =>
          idx === prev.length - 1 ? { ...item, binOrganic: 0, binNonOrganic: 0 } : item
        )
      );
      toast.success("Bins emptied successfully!");
    } catch (err) {
      toast.error("Failed to empty bins: " + err.message);
    } finally {
      setIsBinEmpty(false);
    }
  }

  function handleShutdown() {
    if (!window.confirm("Shut down the robot? This will stop all operations.")) return;
    setShuttingDown(true);
    robotApi
      .shutdown()
      .then(() => toast.success("Shutdown command sent to robot"))
      .catch((err) => toast.error("Shutdown failed: " + err.message))
      .finally(() => setShuttingDown(false));
  }

  const userInitial = user?.name ? user.name[0].toUpperCase() : "?";

  const NAV = [
    { id: "overview", icon: "◈", label: "Overview" },
    { id: "bins",     icon: "⬡", label: "Bins" },
    { id: "system",   icon: "⊞", label: "System" },
    { id: "control",  icon: "⊕", label: "Control" },
    { id: "tracking", icon: "◉", label: "Tracking" },
  ];

  return (
    <div className="app-shell">
      {apiError && (
        <div className="api-error-banner">
          ⚠ Telemetry connection lost — retrying...
        </div>
      )}

      <aside className="sidebar">
        <div className="sidebar-logo">
          <div className="sidebar-logo-icon">GG</div>
          <div>
            <div className="sidebar-logo-text">Green Guardian</div>
            <div className="sidebar-logo-sub">Robot Dashboard</div>
          </div>
        </div>

        <nav className="sidebar-nav">
          {NAV.map((n) => (
            <button
              key={n.id}
              className={`nav-item${activeTab === n.id ? " active" : ""}`}
              onClick={() => setActiveTab(n.id)}
            >
              <span className="nav-icon">{n.icon}</span>
              {n.label}
            </button>
          ))}

          <div className="nav-divider" />

          <button className="nav-item" onClick={() => navigate("/manage")}>
            <span className="nav-icon">⚙</span>
            Manage
          </button>
        </nav>

        <div className="sidebar-bottom">
          <div className="sidebar-user">
            <div className="user-avatar">{userInitial}</div>
            <span className="user-name">{user?.name}</span>
          </div>
          <button
            className="shutdown-btn"
            onClick={handleShutdown}
            disabled={shuttingDown}
          >
            {shuttingDown ? "Shutting down..." : "⏻ Shutdown Robot"}
          </button>
          <button className="logout-btn-sidebar" onClick={logout}>
            ↩ Logout
          </button>
        </div>
      </aside>

      <main className="dashboard">
        <div className="dashboard-header">
          <h1>
            {NAV.find((n) => n.id === activeTab)?.icon}{" "}
            {NAV.find((n) => n.id === activeTab)?.label ||
              (activeTab === "tracking" ? "Tracking" : "")}
          </h1>
          <div className="header-right">
            <div className={`live-indicator${apiError ? " offline" : ""}`}>
              <span className="live-dot" />
              {apiError ? "Offline" : "Live"}
            </div>
          </div>
        </div>

        <div className="dashboard-grid">

          {/* ===== OVERVIEW TAB ===== */}
          {activeTab === "overview" && (
            <>
              {/* Hero Stats */}
              <div className="grid-row grid-row-3">
                <div className="hero-stat-card" style={{ "--hero-accent": "#00d4ff" }}>
                  <p className="hero-stat-label">Speed</p>
                  {last ? (
                    <>
                      <div>
                        <span className="hero-stat-value">{last.speed}</span>
                        <span className="hero-stat-unit"> m/s</span>
                      </div>
                      <p className="hero-stat-sub" style={{ color: last.speed > 7 ? "#ff006e" : last.speed > 4 ? "#ffaa00" : "#39ff14" }}>
                        {last.speed > 7 ? "Fast" : last.speed > 4 ? "Moderate" : last.speed > 0 ? "Slow" : "Stationary"}
                      </p>
                    </>
                  ) : <div className="skeleton" style={{ height: 48, width: 100, marginTop: 12 }} />}
                </div>

                <div className="hero-stat-card" style={{ "--hero-accent": tempColor(last?.cpuTemp || 0) }}>
                  <p className="hero-stat-label">CPU Temp</p>
                  {last ? (
                    <>
                      <div>
                        <span className="hero-stat-value" style={{ color: tempColor(last.cpuTemp) }}>
                          {last.cpuTemp}
                        </span>
                        <span className="hero-stat-unit"> °C</span>
                      </div>
                      <p className="hero-stat-sub" style={{ color: tempColor(last.cpuTemp) }}>
                        {last.cpuTemp < 50 ? "Normal" : last.cpuTemp < 70 ? "Warm" : "Hot"}
                      </p>
                    </>
                  ) : <div className="skeleton" style={{ height: 48, width: 100, marginTop: 12 }} />}
                </div>

                {(() => {
                  const info = throttleInfo(last?.throttled);
                  return (
                    <div className="hero-stat-card" style={{ "--hero-accent": info.color }}>
                      <p className="hero-stat-label">Power Status</p>
                      {last ? (
                        <>
                          <div>
                            <span className="hero-stat-value" style={{ color: info.color }}>
                              {info.symbol}
                            </span>
                          </div>
                          <p className="hero-stat-sub" style={{ color: info.color }}>{info.label}</p>
                          <p style={{ fontSize: 11, color: "var(--text-muted)", marginTop: 2 }}>{info.sub}</p>
                        </>
                      ) : <div className="skeleton" style={{ height: 48, width: 100, marginTop: 12 }} />}
                    </div>
                  );
                })()}
              </div>

              {/* Charts Row */}
              <div className="grid-row grid-row-2">
                <div className="card">
                  <p className="card-title">Speed Over Time</p>
                  <ResponsiveContainer width="100%" height={220}>
                    <LineChart data={history}>
                      <CartesianGrid {...CHART_DARK.grid} />
                      <XAxis dataKey="timeLabel" minTickGap={20} tick={CHART_DARK.tick} axisLine={false} tickLine={false} />
                      <YAxis tick={CHART_DARK.tick} axisLine={false} tickLine={false} />
                      <Tooltip {...CHART_DARK.tooltip} />
                      <Line isAnimationActive={false} type="monotone" dataKey="speed" stroke="#00d4ff" strokeWidth={2} dot={false} name="Speed (m/s)" />
                    </LineChart>
                  </ResponsiveContainer>
                </div>

                <div className="card">
                  <p className="card-title">CPU & RAM Usage</p>
                  <ResponsiveContainer width="100%" height={220}>
                    <LineChart data={history}>
                      <CartesianGrid {...CHART_DARK.grid} />
                      <XAxis dataKey="timeLabel" minTickGap={20} tick={CHART_DARK.tick} axisLine={false} tickLine={false} />
                      <YAxis domain={[0, 100]} tick={CHART_DARK.tick} axisLine={false} tickLine={false} />
                      <Tooltip {...CHART_DARK.tooltip} />
                      <Legend wrapperStyle={{ fontSize: 11, color: "#475569" }} />
                      <Line isAnimationActive={false} type="monotone" dataKey="cpuUsage" stroke="#39ff14" strokeWidth={2} dot={false} name="CPU %" />
                      <Line isAnimationActive={false} type="monotone" dataKey="ramUsage" stroke="#a855f7" strokeWidth={2} dot={false} name="RAM %" />
                    </LineChart>
                  </ResponsiveContainer>
                </div>
              </div>
            </>
          )}

          {/* ===== BINS TAB ===== */}
          {activeTab === "bins" && (
            <>
              <div className="bins-grid">
                <div className="bin-donut-card">
                  <p className="card-title">Organic Bin</p>
                  <DonutGauge value={last?.binOrganic || 0} color="#39ff14" />
                  <div className="donut-label-group">
                    <div className="donut-percent" style={{ color: "#39ff14" }}>
                      {last?.binOrganic ?? "--"}%
                    </div>
                    <div className="donut-name">Organic Waste</div>
                  </div>
                </div>

                <div className="bin-donut-card">
                  <p className="card-title">Non-Organic Bin</p>
                  <DonutGauge value={last?.binNonOrganic || 0} color="#ffaa00" />
                  <div className="donut-label-group">
                    <div className="donut-percent" style={{ color: "#ffaa00" }}>
                      {last?.binNonOrganic ?? "--"}%
                    </div>
                    <div className="donut-name">Non-Organic Waste</div>
                  </div>
                </div>
              </div>

              <div className="bins-bottom">
                {last && (last.binOrganic > 90 || last.binNonOrganic > 90) && (
                  <div className="bin-warning">
                    ⚠ Warning: One or more bins are almost full ({">"} 90%)
                  </div>
                )}
                <div className="card bin-actions-card">
                  <p className="card-title">Bin Actions</p>
                  <p style={{ fontSize: 13, color: "var(--text-muted)", marginBottom: 16 }}>
                    Command the robot to empty and reset the bins
                  </p>
                  <button
                    className="empty-bin-btn"
                    onClick={handleEmptyBin}
                    disabled={isBinEmpty}
                  >
                    {isBinEmpty ? "⟳ Emptying..." : "⬡ Empty Bins"}
                  </button>
                </div>
              </div>
            </>
          )}

          {/* ===== SYSTEM TAB ===== */}
          {activeTab === "system" && (
            <>
              <div className="grid-row grid-row-4">
                <div className="metric-card">
                  <p className="metric-label">CPU Temp</p>
                  <ArcGauge value={last?.cpuTemp || 0} max={100} color={tempColor(last?.cpuTemp || 0)} />
                  <div className="metric-value" style={{ color: tempColor(last?.cpuTemp || 0) }}>
                    {last?.cpuTemp ?? "--"}
                    <span className="metric-unit">°C</span>
                  </div>
                </div>

                <div className="metric-card">
                  <p className="metric-label">CPU Usage</p>
                  <ArcGauge value={last?.cpuUsage || 0} max={100} color={usageColor(last?.cpuUsage || 0)} />
                  <div className="metric-value" style={{ color: usageColor(last?.cpuUsage || 0) }}>
                    {last?.cpuUsage ?? "--"}
                    <span className="metric-unit">%</span>
                  </div>
                </div>

                <div className="metric-card">
                  <p className="metric-label">RAM Usage</p>
                  <ArcGauge value={last?.ramUsage || 0} max={100} color="#a855f7" />
                  <div className="metric-value" style={{ color: "#a855f7" }}>
                    {last?.ramUsage ?? "--"}
                    <span className="metric-unit">%</span>
                  </div>
                </div>

                <div className="metric-card">
                  <p className="metric-label">Fan Speed</p>
                  <ArcGauge value={Math.min(last?.fanSpeed || 0, 5000)} max={5000} color="#00d4ff" />
                  <div className="metric-value" style={{ color: "#00d4ff" }}>
                    {last?.fanSpeed ?? "--"}
                    <span className="metric-unit" style={{ fontSize: 11 }}> RPM</span>
                  </div>
                </div>
              </div>

              {/* Power Status Row */}
              {(() => {
                const info = throttleInfo(last?.throttled);
                return (
                  <div className="card">
                    <p className="card-title">Power Status</p>
                    <div style={{ display: "flex", alignItems: "center", gap: 20 }}>
                      <span style={{ fontSize: 42, fontWeight: 700, color: info.color, lineHeight: 1 }}>
                        {info.symbol}
                      </span>
                      <div>
                        <div style={{ fontSize: 18, fontWeight: 700, color: info.color }}>{info.label}</div>
                        <div style={{ fontSize: 12, color: "var(--text-muted)", marginTop: 4 }}>{info.sub}</div>
                        {last?.throttled >= 0 && (
                          <div style={{ fontSize: 11, color: "var(--text-muted)", marginTop: 6, fontFamily: "monospace" }}>
                            throttled=0x{(last.throttled).toString(16).toUpperCase()}
                          </div>
                        )}
                      </div>
                    </div>
                  </div>
                );
              })()}

              {/* System History Chart */}
              <div className="card">
                <p className="card-title">System History</p>
                <ResponsiveContainer width="100%" height={260}>
                  <LineChart data={history}>
                    <CartesianGrid {...CHART_DARK.grid} />
                    <XAxis dataKey="timeLabel" minTickGap={20} tick={CHART_DARK.tick} axisLine={false} tickLine={false} />
                    <YAxis domain={[0, 100]} tick={CHART_DARK.tick} axisLine={false} tickLine={false} />
                    <Tooltip {...CHART_DARK.tooltip} />
                    <Legend wrapperStyle={{ fontSize: 11, color: "#475569" }} />
                    <Line isAnimationActive={false} type="monotone" dataKey="cpuTemp" stroke="#ff006e" strokeWidth={2} dot={false} name="CPU Temp °C" />
                    <Line isAnimationActive={false} type="monotone" dataKey="cpuUsage" stroke="#39ff14" strokeWidth={2} dot={false} name="CPU %" />
                    <Line isAnimationActive={false} type="monotone" dataKey="ramUsage" stroke="#a855f7" strokeWidth={2} dot={false} name="RAM %" />
                  </LineChart>
                </ResponsiveContainer>
              </div>
            </>
          )}

          {/* ===== CONTROL TAB ===== */}
          {activeTab === "control" && <ManualControl />}

          {/* ===== TRACKING TAB ===== */}
          {activeTab === "tracking" && (
            <div className="card tracking-card">
              <p className="card-title">◉ RViz2 — Live Tracking (via noVNC)</p>
              <div className="novnc-wrapper">
                <iframe
                  src={NOVNC_URL}
                  title="RViz2 via noVNC"
                  className="novnc-iframe"
                  allowFullScreen
                />
              </div>
              <p className="novnc-hint">
                Streaming RViz2 from the laptop via noVNC. Set{" "}
                <code>VITE_NOVNC_URL</code> in <code>.env.local</code> to change the address.
              </p>
            </div>
          )}

        </div>
      </main>
    </div>
  );
}
