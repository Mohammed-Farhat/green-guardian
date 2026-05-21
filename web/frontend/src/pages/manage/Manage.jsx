import { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { toast } from "react-toastify";
import { useAuth } from "../../context/AuthContext";
import { telemetryApi, robotApi } from "../../services/api";
import "../dashboard/dashboard.css";
import "./Manage.css";

export default function Manage() {
  const { user, logout } = useAuth();
  const navigate = useNavigate();
  const [last, setLast] = useState(null);
  const [rosConnected, setRosConnected] = useState(false);
  const [shuttingDown, setShuttingDown] = useState(false);
  const [emptyingBins, setEmptyingBins] = useState(false);

  useEffect(() => {
    async function fetchStatus() {
      try {
        const res = await telemetryApi.getLatest();
        setLast(res.data);
        setRosConnected(res.source === "live");
      } catch {
        setRosConnected(false);
      }
    }

    fetchStatus();
    const id = setInterval(fetchStatus, 5000);
    return () => clearInterval(id);
  }, []);

  function handleShutdown() {
    if (!window.confirm("Shut down the robot? This will stop all operations.")) return;
    setShuttingDown(true);
    robotApi
      .shutdown()
      .then(() => toast.success("Shutdown command sent"))
      .catch((err) => toast.error("Shutdown failed: " + err.message))
      .finally(() => setShuttingDown(false));
  }

  async function handleEmptyBins() {
    setEmptyingBins(true);
    try {
      await robotApi.emptyBins();
      toast.success("Bins emptied successfully!");
    } catch (err) {
      toast.error("Failed: " + err.message);
    } finally {
      setEmptyingBins(false);
    }
  }

  const userInitial = user?.name ? user.name[0].toUpperCase() : "?";

  return (
    <div className="app-shell">
      <aside className="sidebar">
        <div className="sidebar-logo">
          <div className="sidebar-logo-icon">GG</div>
          <div>
            <div className="sidebar-logo-text">Green Guardian</div>
            <div className="sidebar-logo-sub">Robot Dashboard</div>
          </div>
        </div>

        <nav className="sidebar-nav">
          <button className="nav-item" onClick={() => navigate("/")}>
            <span className="nav-icon">◈</span>
            Dashboard
          </button>
          <button className="nav-item active">
            <span className="nav-icon">⚙</span>
            Manage
          </button>
        </nav>

        <div className="sidebar-bottom">
          <div className="sidebar-user">
            <div className="user-avatar">{userInitial}</div>
            <span className="user-name">{user?.name}</span>
          </div>
          <button className="logout-btn-sidebar" onClick={logout}>
            ↩ Logout
          </button>
        </div>
      </aside>

      <div className="manage-content">
        <h1>⚙ Manage</h1>

        <div className="manage-grid">
          <div className="manage-card">
            <h3>Robot Status</h3>
            <div className="robot-status">
              <span className={`status-dot ${rosConnected ? "online" : "offline"}`} />
              <span>{rosConnected ? "Online — Live Data" : "Offline"}</span>
            </div>

            {last && (
              <ul className="robot-info-list">
                <li><span>Speed</span><strong>{Number((last.speed || 0).toFixed(2))} m/s</strong></li>
                <li><span>CPU Temp</span><strong>{Math.round(last.cpuTemp || 0)} °C</strong></li>
                <li><span>CPU Usage</span><strong>{Math.round(last.cpuUsage || 0)}%</strong></li>
                <li><span>RAM Usage</span><strong>{Math.round(last.ramUsage || 0)}%</strong></li>
                <li><span>Power</span><strong>{last.throttled === 0 ? "OK" : last.throttled > 0 ? `0x${last.throttled.toString(16).toUpperCase()}` : "--"}</strong></li>
                <li><span>Organic Bin</span><strong>{Math.round(last.binOrganic || 0)}%</strong></li>
                <li><span>Non-Organic Bin</span><strong>{Math.round(last.binNonOrganic || 0)}%</strong></li>
              </ul>
            )}
          </div>

          <div className="manage-card">
            <h3>Robot Controls</h3>
            <div className="manage-actions">
              <button
                className="manage-btn secondary"
                onClick={handleEmptyBins}
                disabled={emptyingBins}
              >
                {emptyingBins ? "⟳ Emptying..." : "⬡ Empty Bins"}
              </button>
              <button
                className="manage-btn danger"
                onClick={handleShutdown}
                disabled={shuttingDown}
              >
                {shuttingDown ? "Shutting down..." : "⏻ Shutdown Robot"}
              </button>
            </div>
          </div>

          <div className="manage-card">
            <h3>Account</h3>
            <div className="user-info-card">
              <p><strong>{user?.name}</strong></p>
              <p>{user?.email}</p>
              <p>Role: <strong>{user?.role}</strong></p>
              <button className="logout-btn" onClick={logout}>
                ↩ Log Out
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
