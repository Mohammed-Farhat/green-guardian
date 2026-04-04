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
} from "recharts";
import "./Dashboard.css";
import { telemetryApi, robotApi } from "../../services/api";
import { useAuth } from "../../context/AuthContext";

function getTempColor(t) {
  if (t < 50) return "#4caf50";
  if (t < 70) return "#ff9800";
  return "#f44336";
}

function getTempLabel(t) {
  if (t < 50) return "OK";
  if (t < 70) return "Warm";
  return "Hot";
}

export default function Dashboard() {
  const { user, logout } = useAuth();
  const navigate = useNavigate();
  const [last, setLast] = useState(null);
  const [history, setHistory] = useState([]);
  const [activeTab, setActiveTab] = useState("overview");
  const [isBinEmpty, setIsBinEmpty] = useState(false);
  const [batteryAlerted, setBatteryAlerted] = useState(false);
  const [apiError, setApiError] = useState(false);
  const [shuttingDown, setShuttingDown] = useState(false);

  // Battery alert
  useEffect(() => {
    if (!last) return;
    if (last.battery <= 20 && !batteryAlerted) {
      toast.error("Battery critical (20%) - Robot needs charging!");
      setBatteryAlerted(true);
    }
    if (last.battery > 25 && batteryAlerted) {
      setBatteryAlerted(false);
    }
  }, [last, batteryAlerted]);

  // Fetch history on mount
  useEffect(() => {
    async function loadHistory() {
      try {
        const data = await telemetryApi.getHistory(30);
        const reversed = data.reverse();
        const mapped = reversed.map((t) => ({
          speed: Number((t.speed || 0).toFixed(2)),
          cpuTemp: Math.round(t.cpuTemp || 0),
          cpuUsage: Math.round(t.cpuUsage || 0),
          ramUsage: Math.round(t.ramUsage || 0),
          fanSpeed: Math.round(t.fanSpeed || 0),
          binOrganic: Math.round(t.binOrganic || 0),
          binNonOrganic: Math.round(t.binNonOrganic || 0),
          battery: Math.round(t.battery || 0),
          timestamp: new Date(t.timestamp).getTime(),
          timeLabel: new Date(t.timestamp).toLocaleTimeString(),
        }));
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
        const t = res.data;

        const sample = {
          speed: Number((t.speed || 0).toFixed(2)),
          cpuTemp: Math.round(t.cpuTemp || 0),
          cpuUsage: Math.round(t.cpuUsage || 0),
          ramUsage: Math.round(t.ramUsage || 0),
          fanSpeed: Math.round(t.fanSpeed || 0),
          binOrganic: Math.round(t.binOrganic || 0),
          binNonOrganic: Math.round(t.binNonOrganic || 0),
          battery: Math.round(t.battery || 0),
          timestamp: Date.now(),
          timeLabel: new Date().toLocaleTimeString(),
        };

        setLast(sample);
        setHistory((prev) => [...prev, sample].slice(-30));
        setApiError(false);
      } catch (err) {
        if (!apiError) {
          console.error("Telemetry poll failed:", err.message);
          setApiError(true);
        }
      }
    };

    const id = setInterval(poll, 3000);
    poll();
    return () => clearInterval(id);
  }, []);

  function handleEmptyBin() {
    setIsBinEmpty(true);
    robotApi.emptyBins().catch((err) => {
      toast.error("Failed to empty bins: " + err.message);
    });
    setTimeout(() => {
      setIsBinEmpty(false);
      setHistory((prev) =>
        prev.map((item, idx) =>
          idx === prev.length - 1
            ? { ...item, binOrganic: 0, binNonOrganic: 0 }
            : item
        )
      );
      setLast((prev) =>
        prev ? { ...prev, binOrganic: 0, binNonOrganic: 0 } : null
      );
    }, 2000);
  }

  function handleShutdown() {
    if (!window.confirm("Are you sure you want to turn off the robot?")) return;
    setShuttingDown(true);
    robotApi
      .shutdown()
      .then(() => toast.success("Shutdown command sent"))
      .catch((err) => toast.error("Shutdown failed: " + err.message))
      .finally(() => setShuttingDown(false));
  }

  return (
    <div className="app-shell">
      <aside className="sidebar">
        <div className="sidebar-logo">GG</div>
        <nav className="sidebar-nav">
          <button
            className={`nav-item ${activeTab === "overview" ? "active" : ""}`}
            onClick={() => setActiveTab("overview")}
          >
            Overview
          </button>
          <button
            className={`nav-item ${activeTab === "bins" ? "active" : ""}`}
            onClick={() => setActiveTab("bins")}
          >
            Bins
          </button>
          <button
            className={`nav-item ${activeTab === "system" ? "active" : ""}`}
            onClick={() => setActiveTab("system")}
          >
            System
          </button>
          <button
            className={`nav-item ${activeTab === "tracking" ? "active" : ""}`}
            onClick={() => setActiveTab("tracking")}
          >
            Tracking
          </button>
          <button
            className="nav-item"
            onClick={() => navigate("/manage")}
          >
            Manage
          </button>
        </nav>
        <div className="sidebar-bottom">
          <span className="sidebar-user">{user?.name}</span>
          <button
            className="shutdown-btn"
            onClick={handleShutdown}
            disabled={shuttingDown}
            title="Turn off the robot"
          >
            {shuttingDown ? "..." : "Shutdown"}
          </button>
          <button
            className="logout-btn-sidebar"
            onClick={logout}
            title="Log out"
          >
            Logout
          </button>
        </div>
      </aside>

      <main className="dashboard">
        <div className="dashboard-header">
          <h1>Green Guardian Dashboard</h1>
        </div>

        <div className="dashboard-grid">
          {/* ===== Overview Tab ===== */}
          {activeTab === "overview" && (
            <>
              <div className="card wide-card gauges-card">
                {last ? (
                  <div className="gauges-row">
                    {/* Thermometer */}
                    <div className="gauge-item">
                      <h3>CPU Temp</h3>
                      <div className="thermo-container">
                        <div className="thermometer">
                          <div className="thermo-track">
                            <div className="thermo-scale">
                              <span>100°</span>
                              <span>70°</span>
                              <span>50°</span>
                              <span>0°</span>
                            </div>
                            <div className="thermo-tube">
                              <div
                                className="thermo-fill"
                                style={{
                                  height: `${Math.min(last.cpuTemp, 100)}%`,
                                  backgroundColor: getTempColor(last.cpuTemp),
                                }}
                              />
                              <div className="thermo-zone hot-zone" />
                              <div className="thermo-zone warm-zone" />
                            </div>
                          </div>
                          <div className="thermo-bulb" style={{ backgroundColor: getTempColor(last.cpuTemp) }}>
                            <span className="thermo-bulb-value">{last.cpuTemp}°</span>
                          </div>
                        </div>
                      </div>
                      <span className="gauge-value" style={{ color: getTempColor(last.cpuTemp) }}>
                        {last.cpuTemp}°C
                      </span>
                      <span className="gauge-label">{getTempLabel(last.cpuTemp)}</span>
                    </div>

                    {/* Speedometer */}
                    <div className="gauge-item">
                      <h3>Speed</h3>
                      <div className="speedo-wrapper">
                        <svg viewBox="0 0 200 130" className="speedo-svg">
                          {/* Background arc */}
                          <path
                            d="M 20 120 A 80 80 0 0 1 180 120"
                            fill="none"
                            stroke="#e8e8e8"
                            strokeWidth="14"
                            strokeLinecap="round"
                          />
                          {/* Green zone 0-4 */}
                          <path
                            d="M 20 120 A 80 80 0 0 1 52.15 52.15"
                            fill="none"
                            stroke="#4caf50"
                            strokeWidth="14"
                            strokeLinecap="round"
                          />
                          {/* Orange zone 4-7 */}
                          <path
                            d="M 52.15 52.15 A 80 80 0 0 1 100 40"
                            fill="none"
                            stroke="#ff9800"
                            strokeWidth="14"
                          />
                          <path
                            d="M 100 40 A 80 80 0 0 1 131.76 47.61"
                            fill="none"
                            stroke="#ff9800"
                            strokeWidth="14"
                          />
                          {/* Red zone 7-10 */}
                          <path
                            d="M 131.76 47.61 A 80 80 0 0 1 180 120"
                            fill="none"
                            stroke="#f44336"
                            strokeWidth="14"
                            strokeLinecap="round"
                          />
                          {/* Needle */}
                          <line
                            x1="100"
                            y1="120"
                            x2={100 + 68 * Math.cos(Math.PI + (Math.min(last.speed, 10) / 10) * Math.PI)}
                            y2={120 + 68 * Math.sin(Math.PI + (Math.min(last.speed, 10) / 10) * Math.PI)}
                            stroke="#c33"
                            strokeWidth="2.5"
                            strokeLinecap="round"
                          />
                          {/* Center dot */}
                          <circle cx="100" cy="120" r="6" fill="#c33" />
                          <circle cx="100" cy="120" r="3" fill="#fff" />
                        </svg>
                      </div>
                      <span className="gauge-value">{last.speed} m/s</span>
                    </div>

                    {/* CPU Usage bar */}
                    <div className="gauge-item">
                      <h3>CPU Usage</h3>
                      <div className="usage-bar-wrapper">
                        <div className="usage-bar-track">
                          <div
                            className="usage-bar-fill"
                            style={{
                              height: `${last.cpuUsage}%`,
                              backgroundColor:
                                last.cpuUsage < 60 ? "#4caf50" : last.cpuUsage < 85 ? "#ff9800" : "#f44336",
                            }}
                          />
                        </div>
                        <div className="usage-bar-labels">
                          <span>100%</span>
                          <span>75%</span>
                          <span>50%</span>
                          <span>25%</span>
                          <span>0%</span>
                        </div>
                      </div>
                      <span className="gauge-value">{last.cpuUsage}%</span>
                    </div>

                    {/* RAM Usage bar */}
                    <div className="gauge-item">
                      <h3>RAM Usage</h3>
                      <div className="usage-bar-wrapper">
                        <div className="usage-bar-track">
                          <div
                            className="usage-bar-fill"
                            style={{
                              height: `${last.ramUsage}%`,
                              backgroundColor:
                                last.ramUsage < 60 ? "#2196f3" : last.ramUsage < 85 ? "#ff9800" : "#f44336",
                            }}
                          />
                        </div>
                        <div className="usage-bar-labels">
                          <span>100%</span>
                          <span>75%</span>
                          <span>50%</span>
                          <span>25%</span>
                          <span>0%</span>
                        </div>
                      </div>
                      <span className="gauge-value">{last.ramUsage}%</span>
                    </div>
                  </div>
                ) : (
                  <p>Loading...</p>
                )}
              </div>

              <div className="card wide-card">
                <h3>Speed Over Time</h3>
                <ResponsiveContainer width="100%" height={260}>
                  <LineChart data={history}>
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis dataKey="timeLabel" minTickGap={20} />
                    <YAxis />
                    <Tooltip />
                    <Line
                      isAnimationActive={false}
                      type="monotone"
                      dataKey="speed"
                      stroke="#2196f3"
                      strokeWidth={2}
                      dot={false}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>
            </>
          )}

          {/* ===== Bins Tab ===== */}
          {activeTab === "bins" && (
            <>
              <div className="card big-card">
                <h3>Current Bin Status</h3>
                {last ? (
                  <div style={{ textAlign: "center", padding: "20px" }}>
                    <div className="bin-wrapper" style={{ alignItems: "center" }}>
                      <p>Organic:</p>
                      <div className="bin-bar" style={{ height: "30px" }}>
                        <div
                          className="bin-fill"
                          style={{
                            width: `${last.binOrganic}%`,
                            backgroundColor: "#4caf50",
                          }}
                        />
                      </div>
                      <p>{last.binOrganic}% Full</p>

                      <p>Non-Organic:</p>
                      <div className="bin-bar" style={{ height: "30px" }}>
                        <div
                          className="bin-fill"
                          style={{
                            width: `${last.binNonOrganic}%`,
                            backgroundColor: "#ff9800",
                          }}
                        />
                      </div>
                      <p>{last.binNonOrganic}% Full</p>

                      {(last.binOrganic > 90 || last.binNonOrganic > 90) && (
                        <p style={{ color: "red", fontWeight: "bold" }}>
                          Warning: Bin Almost Full!
                        </p>
                      )}
                    </div>
                  </div>
                ) : (
                  <p>Loading...</p>
                )}
              </div>

              <div className="card small-card empty-bin-card">
                <h3>Bin Actions</h3>
                <p
                  style={{
                    fontSize: "14px",
                    color: "#666",
                    marginBottom: "16px",
                  }}
                >
                  Command the robot to empty the bins
                </p>
                <button
                  className="empty-bin-btn"
                  onClick={handleEmptyBin}
                  disabled={isBinEmpty}
                  title="Command robot to empty the bins"
                >
                  {isBinEmpty ? "Emptying..." : "Empty Bins"}
                </button>
              </div>
            </>
          )}

          {/* ===== System Tab ===== */}
          {activeTab === "system" && (
            <>
              <div className="card wide-card">
                <h3>CPU Temp / CPU Usage / RAM Usage Over Time</h3>
                <ResponsiveContainer width="100%" height={280}>
                  <LineChart data={history}>
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis dataKey="timeLabel" minTickGap={20} />
                    <YAxis domain={[0, 100]} />
                    <Tooltip />
                    <Line
                      isAnimationActive={false}
                      type="monotone"
                      dataKey="cpuTemp"
                      stroke="#9c27b0"
                      strokeWidth={2}
                      dot={false}
                      name="CPU Temp (°C)"
                    />
                    <Line
                      isAnimationActive={false}
                      type="monotone"
                      dataKey="cpuUsage"
                      stroke="#4caf50"
                      strokeWidth={2}
                      dot={false}
                      name="CPU Usage (%)"
                    />
                    <Line
                      isAnimationActive={false}
                      type="monotone"
                      dataKey="ramUsage"
                      stroke="#2196f3"
                      strokeWidth={2}
                      dot={false}
                      name="RAM Usage (%)"
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>

              <div className="card small-card">
                <h3>CPU & System</h3>
                {last ? (
                  <ul className="metrics-list">
                    <li>
                      <span>CPU Temp</span>
                      <strong>{last.cpuTemp}°C</strong>
                    </li>
                    <li>
                      <span>CPU Usage</span>
                      <strong>{last.cpuUsage}%</strong>
                    </li>
                    <li>
                      <span>RAM Usage</span>
                      <strong>{last.ramUsage}%</strong>
                    </li>
                    <li>
                      <span>Fan Speed</span>
                      <strong>{last.fanSpeed} RPM</strong>
                    </li>
                  </ul>
                ) : (
                  <p>Loading...</p>
                )}
              </div>

              <div className="card small-card">
                <h3>Battery Level</h3>
                {last ? (
                  <div style={{ padding: "16px", textAlign: "center" }}>
                    <div
                      className="bin-bar"
                      style={{
                        width: "100%",
                        height: "30px",
                        backgroundColor: "#ddd",
                        borderRadius: "6px",
                        overflow: "hidden",
                      }}
                    >
                      <div
                        className="bin-fill"
                        style={{
                          width: `${last.battery}%`,
                          height: "100%",
                          backgroundColor:
                            last.battery > 50
                              ? "#4caf50"
                              : last.battery > 20
                              ? "#ff9800"
                              : "#f44336",
                          transition: "width 0.3s ease",
                        }}
                      />
                    </div>
                    <p style={{ marginTop: "8px", fontWeight: "600" }}>
                      {last.battery}%
                    </p>
                  </div>
                ) : (
                  <p>Loading...</p>
                )}
              </div>
            </>
          )}

          {/* ===== Tracking Tab (noVNC / RViz2) ===== */}
          {activeTab === "tracking" && (
            <div className="card wide-card tracking-card">
              <h3>Tracking - RViz2 (via noVNC)</h3>
              <div className="novnc-wrapper">
                <iframe
                  src="http://localhost:6080/vnc.html?autoconnect=true&resize=scale"
                  title="RViz2 via noVNC"
                  className="novnc-iframe"
                  allowFullScreen
                />
              </div>
              <p className="novnc-hint">
                Streaming RViz2 from the laptop via noVNC. Make sure noVNC is
                running on port 6080.
              </p>
            </div>
          )}
        </div>
      </main>
    </div>
  );
}
