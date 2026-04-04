import { useEffect, useState } from "react";
import { toast } from "react-toastify";
import {
  PieChart,
  Pie,
  Cell,
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

const TEMP_COLORS = ["#4caf50", "#ff9800", "#f44336"];

export default function Dashboard() {
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

  const tempPieData = (() => {
    if (!last) return [];
    const t = last.cpuTemp;
    return [
      { name: "OK", value: t < 50 ? 1 : 0 },
      { name: "Warm", value: t >= 50 && t < 70 ? 1 : 0 },
      { name: "Hot", value: t >= 70 ? 1 : 0 },
    ];
  })();

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
        </nav>
        <div className="sidebar-bottom">
          <button
            className="shutdown-btn"
            onClick={handleShutdown}
            disabled={shuttingDown}
            title="Turn off the robot"
          >
            {shuttingDown ? "..." : "Shutdown"}
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
              <div className="card big-card">
                <h3>CPU Temperature</h3>
                {last ? (
                  <>
                    <div className="card-main-row">
                      <div className="chart-wrapper">
                        <ResponsiveContainer width="100%" height={240}>
                          <PieChart>
                            <Pie
                              data={tempPieData}
                              innerRadius={70}
                              outerRadius={100}
                              paddingAngle={3}
                              dataKey="value"
                              isAnimationActive={false}
                            >
                              {tempPieData.map((entry, index) => (
                                <Cell key={entry.name} fill={TEMP_COLORS[index]} />
                              ))}
                            </Pie>
                          </PieChart>
                        </ResponsiveContainer>
                      </div>
                      <div className="stats-column">
                        <p>
                          CPU Temp: <strong>{last.cpuTemp} °C</strong>
                        </p>
                        <p>
                          Speed: <strong>{last.speed} m/s</strong>
                        </p>
                      </div>
                    </div>
                    <div className="legend">
                      <span className="dot ok" /> OK (&lt; 50°C)
                      <span className="dot warm" /> Warm (50–69°C)
                      <span className="dot hot" /> Hot (&ge; 70°C)
                    </div>
                  </>
                ) : (
                  <p>Loading...</p>
                )}
              </div>

              <div className="card small-card">
                <h3>Current Metrics</h3>
                {last ? (
                  <ul className="metrics-list">
                    <li>
                      <span>CPU Temperature</span>
                      <strong>{last.cpuTemp} °C</strong>
                    </li>
                    <li>
                      <span>Speed</span>
                      <strong>{last.speed} m/s</strong>
                    </li>
                    <li>
                      <span>Organic Bin</span>
                      <strong>{last.binOrganic} %</strong>
                    </li>
                    <li>
                      <span>Non-Organic Bin</span>
                      <strong>{last.binNonOrganic} %</strong>
                    </li>
                    <li>
                      <span>Last Update</span>
                      <strong>
                        {new Date(last.timestamp).toLocaleTimeString()}
                      </strong>
                    </li>
                  </ul>
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
                <h3>CPU Temperature Over Time</h3>
                <ResponsiveContainer width="100%" height={280}>
                  <LineChart data={history}>
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis dataKey="timeLabel" minTickGap={20} />
                    <YAxis domain={[30, 100]} />
                    <Tooltip />
                    <Line
                      isAnimationActive={false}
                      type="monotone"
                      dataKey="cpuTemp"
                      stroke="#9c27b0"
                      strokeWidth={2}
                      dot={false}
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
