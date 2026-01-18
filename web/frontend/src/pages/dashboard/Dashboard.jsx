import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { toast } from 'react-toastify';
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
import RobotMap from "../../services/RobotMaps";

const TEMP_COLORS = ["#4caf50", "#ff9800", "#f44336"]; // ok / warm / hot

// 🔧 Simulate robot + RPi + GPS data


function simulateRobotData(
  prev = {
    speed: 0,
    temperature: 25,
    binOrganic: 40,
    binNonOrganic: 35,
    cpuTemp: 45,
    fanSpeed: 30,
    humidity: 60,
    battery: 20,
    lat: 33.675637,
    lng: 35.465756,
  }
) {
  const speed = Math.max(0, prev.speed + (Math.random() * 4 - 2));
  const temperature = 25 + Math.sin(Date.now() / 5000) * 5 + Math.random();
  const binOrganic = Math.max(0, Math.min(100, prev.binOrganic + Math.random() * 3 - 1.5));
  const binNonOrganic = Math.max(0, Math.min(100, prev.binNonOrganic + Math.random() * 3 - 1.5));
  const cpuTemp = Math.max(40, Math.min(85, prev.cpuTemp + (Math.random() * 4 - 2)));
  const fanSpeed = Math.max(0, Math.min(100, prev.fanSpeed + (Math.random() * 10 - 5)));
  const humidity = Math.max(30, Math.min(90, prev.humidity + (Math.random() * 3 - 1.5)));
  const battery = Math.max(0, Math.min(100, prev.battery - Math.random() * 0.5));

  const lat = prev.lat + (Math.random() - 0.5) * 0.0005;
  const lng = prev.lng + (Math.random() - 0.5) * 0.0005;

  return {
    timestamp: Date.now(),
    speed: Number(speed.toFixed(2)),
    temperature: Number(temperature.toFixed(2)),
    binOrganic: Math.round(binOrganic),
    binNonOrganic: Math.round(binNonOrganic),
    cpuTemp: Math.round(cpuTemp),
    fanSpeed: Math.round(fanSpeed),
    humidity: Math.round(humidity),
    battery: Math.round(battery),
    lat: Number(lat.toFixed(6)),
    lng: Number(lng.toFixed(6)),
  };
}

export default function Dashboard() {
  
  const navigate = useNavigate();
  const [last, setLast] = useState(null);
  const [history, setHistory] = useState([]);
  const [activeTab, setActiveTab] = useState("dashboard");
  const [isBinEmpty, setIsBinEmpty] = useState(false);
  const [batteryAlerted, setBatteryAlerted] = useState(false);
  useEffect(() => {
  if (!last) return;

  if (last.battery <= 20 && !batteryAlerted) {
    toast.error("🔋 Battery critical (20%) – Robot needs charging!");
    setBatteryAlerted(true);
  }

  // reset alert if battery goes back up (charging simulation)
  if (last.battery > 25 && batteryAlerted) {
    setBatteryAlerted(false);
  }
}, [last, batteryAlerted]);
  useEffect(() => {
    let lastSample = {
      speed: 0,
      temperature: 25,
      binOrganic: 40,
      binNonOrganic: 35,
      cpuTemp: 45,
      fanSpeed: 30,
      humidity: 60,
      battery: 20,
      lat: 33.675637,
      lng: 35.465756,
    };

    const update = () => {
      const sample = simulateRobotData(lastSample);
      lastSample = sample;

      const timeLabel = new Date(sample.timestamp).toLocaleTimeString();

      setLast(sample);
      setHistory((prev) => {
        const next = [...prev, { ...sample, timeLabel }];
        return next.slice(-30);
      });
    };

    update();
    const id = setInterval(update, 3000);
    return () => clearInterval(id);
  }, []);

  const tempPieData = (() => {
    if (!last) return [];
    const t = last.temperature;
    return [
      { name: "OK", value: t < 30 ? 1 : 0 },
      { name: "Warm", value: t >= 30 && t < 40 ? 1 : 0 },
      { name: "Hot", value: t >= 40 ? 1 : 0 },
    ];
  })();

  function handleEmptyBin() {
    setIsBinEmpty(true);
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

  return (
    <div className="app-shell">
      <aside className="sidebar">
        <button
          className="sidebar-logo-btn"
          onClick={() => navigate("/robots")}
          aria-label="Go back to robot selection"
          title="Select another robot"
        >
          <div className="sidebar-logo">🤖</div>
        </button>
        <nav className="sidebar-nav">
          <button
            className={`nav-item ${activeTab === "dashboard" ? "active" : ""}`}
            onClick={() => setActiveTab("dashboard")}
          >
            Dashboard
          </button>
          <button
            className={`nav-item ${activeTab === "bin" ? "active" : ""}`}
            onClick={() => setActiveTab("bin")}
          >
            Bin
          </button>
          <button
            className={`nav-item ${activeTab === "speed" ? "active" : ""}`}
            onClick={() => setActiveTab("speed")}
          >
            Speed
          </button>
          <button
            className={`nav-item ${activeTab === "tracking" ? "active" : ""}`}
            onClick={() => setActiveTab("tracking")}
          >
            Tracking
          </button>
          <button
            className={`nav-item ${activeTab === "details" ? "active" : ""}`}
            onClick={() => setActiveTab("details")}
          >
            System Info
          </button>
        </nav>
      </aside>

      <main className="dashboard">
        <div className="dashboard-header">
          <h1>Green Guardian Dashboard</h1>
        </div>

        <div className="dashboard-grid">
          {/* ===== Dashboard Tab ===== */}
          {activeTab === "dashboard" && (
            <>
              <div className="card big-card">
                <h3>Temperature Status</h3>
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
                          Temperature: <strong>{last.temperature} °C</strong>
                        </p>
                        <p>
                          Speed: <strong>{last.speed} m/s</strong>
                        </p>
                      </div>
                    </div>

                    <div className="legend">
                      <span className="dot ok" /> OK (&lt; 30°C)
                      <span className="dot warm" /> Warm (30–39°C)
                      <span className="dot hot" /> Hot (≥ 40°C)
                    </div>
                  </>
                ) : (
                  <p>Loading…</p>
                )}
              </div>

              {/* Metrics Card */}
              <div className="card small-card">
                <h3>Current Metrics</h3>
                {last ? (
                  <ul className="metrics-list">
                    <li>
                      <span>Temperature</span>
                      <strong>{last.temperature} °C</strong>
                    </li>
                    <li>
                      <span>Speed</span>
                      <strong>{last.speed} m/s</strong>
                    </li>
                    <li>
                      <span>Organic Bin Level</span>
                      <strong>{last.binOrganic} %</strong>
                    </li>
                    <li>
                      <span>Non-Organic Bin Level</span>
                      <strong>{last.binNonOrganic} %</strong>
                    </li>
                    <li>
                      <span>Last update</span>
                      <strong>
                        {new Date(last.timestamp).toLocaleTimeString()}
                      </strong>
                    </li>
                  </ul>
                ) : (
                  <p>Loading…</p>
                )}
              </div>

              {/* Speed over time */}
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

          {/* ===== Bin Tab ===== */}
          {activeTab === "bin" && (
            <>
              <div className="card big-card">
                <h3>Current Bin Status</h3>
                {last ? (
                  <div style={{ textAlign: "center", padding: "20px" }}>
                    <div className="bin-wrapper" style={{ alignItems: "center" }}>
                      <p>Organic:</p>
                      <div className="bin-bar" style={{ width: "800px", height: "30px" }}>
                        <div className="bin-fill" style={{ width: `${last.binOrganic}%`, backgroundColor: "#4caf50" }} />
                      </div>
                      <p>{last.binOrganic}% Full</p>

                      <p>Non-Organic:</p>
                      <div className="bin-bar" style={{ width: "800px", height: "30px" }}>
                        <div className="bin-fill" style={{ width: `${last.binNonOrganic}%`, backgroundColor: "#ff9800" }} />
                      </div>
                      <p>{last.binNonOrganic}% Full</p>

                      {(last.binOrganic > 90 || last.binNonOrganic > 90) && (
                        <p style={{ color: "red", fontWeight: "bold" }}>⚠️ Bin Almost Full!</p>
                      )}
                    </div>
                  </div>
                ) : (
                  <p>Loading…</p>
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
                  Force the robot to empty the bins
                </p>
                <button
                  className="empty-bin-btn"
                  onClick={handleEmptyBin}
                  disabled={isBinEmpty}
                  title="Command robot to empty the bins"
                >
                  {isBinEmpty ? "🔄 Emptying..." : "🗑️ Empty Bins"}
                </button>
              </div>
            </>
          )}

          {/* ===== Speed Tab ===== */}
          {activeTab === "speed" && (
            <>
              <div className="card wide-card">
                <h3>Speed History (m/s)</h3>
                <ResponsiveContainer width="100%" height={300}>
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
                      strokeWidth={3}
                      dot={false}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>
              <div className="card big-card">
                <h3>Current Speed</h3>
                {last ? (
                  <div style={{ textAlign: "center", padding: "40px 20px" }}>
                    <div
                      style={{
                        fontSize: "64px",
                        color: "#2196f3",
                        marginBottom: "20px",
                      }}
                    >
                      {last.speed}
                    </div>
                    <p style={{ fontSize: "18px", color: "#666" }}>
                      meters per second
                    </p>
                  </div>
                ) : (
                  <p>Loading…</p>
                )}
              </div>
            </>
          )}

          {/* ===== Tracking Tab ===== */}
          {activeTab === "tracking" && (
            <>
              <div className="card wide-card">
                <h3>Tracking – Video</h3>
                <div
                  className="video-section"
                  style={{
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center",
                    height: "260px",
                    background: "#111827",
                    borderRadius: "12px",
                  }}
                >
                  <div
                    style={{
                      color: "#e5e7eb",
                      textAlign: "center",
                      fontSize: "16px",
                    }}
                  >
                    Live camera stream placeholder
                    <br />
                    <span style={{ fontSize: "13px", color: "#9ca3af" }}>
                      (Embed your RTSP/WebRTC/HTTP stream here)
                    </span>
                  </div>
                </div>
              </div>

              <div className="card wide-card">
                <h3>Tracking – GPS</h3>
                {last ? (
                  <div className="gps-section" style={{ padding: "16px" }}>
                    <div style={{ marginBottom: "16px" }}>
                      <p style={{ marginBottom: "4px", fontSize: "14px" }}>
                        Current Position:
                      </p>
                      <p style={{ fontWeight: "600", fontSize: "18px" }}>
                        {last.lat}, {last.lng}
                      </p>
                      <p style={{ fontSize: "13px", color: "#666" }}>
                        Last update: {new Date(last.timestamp).toLocaleTimeString()}
                      </p>
                    </div>
                    <RobotMap lat={last.lat} lng={last.lng} history={history} />
                  </div>
                ) : (
                  <p>Loading…</p>
                )}
              </div>
            </>
          )}

          {/* ===== System Info Tab ===== */}
          {activeTab === "details" && (
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

              {/* <div className="card wide-card">
                <h3>Fan Speed Over Time (%)</h3>
                <ResponsiveContainer width="100%" height={280}>
                  <LineChart data={history}>
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis dataKey="timeLabel" minTickGap={20} />
                    <YAxis domain={[0, 100]} />
                    <Tooltip />
                    <Line
                      isAnimationActive={false}
                      type="monotone"
                      dataKey="fanSpeed"
                      stroke="#00bcd4"
                      strokeWidth={2}
                      dot={false}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div> */}

              <div className="card small-card">
                <h3>CPU & System</h3>
                {last ? (
                  <ul className="metrics-list">
                    <li>
                      <span>CPU Temp</span>
                      <strong>{last.cpuTemp}°C</strong>
                    </li>
                    <li>
                      <span>Fan Speed</span>
                      <strong>{last.fanSpeed}%</strong>
                    </li>
                    <li>
                      <span>Humidity</span>
                      <strong>{last.humidity}%</strong>
                    </li>
                  </ul>
                ) : (
                  <p>Loading…</p>
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
                    <p style={{ marginTop: "8px", fontWeight: "600" }}>{last.battery}%</p>
                  </div>
                ) : (
                  <p>Loading…</p>
                )}
              </div>


            </>
          )}
        </div>
      </main>
    </div>
  );
}
