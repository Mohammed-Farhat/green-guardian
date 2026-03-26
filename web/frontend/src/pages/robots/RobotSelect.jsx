import { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { toast } from "react-toastify";
import "./RobotSelect.css";
import RobotFleetMap from "../../services/RobotFleetMap";
import { robotApi } from "../../services/api";
import { useAuth } from "../../App";

// Default map coordinates (used when robot has no location data)
const DEFAULT_COORDS = [
  { lat: 33.6758, lng: 35.4655 },
  { lat: 33.6761, lng: 35.466 },
  { lat: 33.6754, lng: 35.4663 },
  { lat: 33.6759, lng: 35.465 },
];

export default function RobotSelect() {
  const [robots, setRobots] = useState([]);
  const [selectedRobot, setSelectedRobot] = useState(null);
  const [loading, setLoading] = useState(true);
  const navigate = useNavigate();
  const { logout } = useAuth();

  useEffect(() => {
    fetchRobots();
  }, []);

  async function fetchRobots() {
    try {
      const data = await robotApi.getAll();
      const mapped = data.map((robot, i) => ({
        id: robot._id,
        name: robot.name,
        status:
          robot.status === "online"
            ? "Active"
            : robot.status === "maintenance"
            ? "Maintenance"
            : "Idle",
        icon: "\u{1F916}",
        location: robot.location || "",
        lat: DEFAULT_COORDS[i % DEFAULT_COORDS.length].lat,
        lng: DEFAULT_COORDS[i % DEFAULT_COORDS.length].lng,
      }));
      setRobots(mapped);
    } catch (err) {
      toast.error("Failed to load robots: " + err.message);
    } finally {
      setLoading(false);
    }
  }

  function handleSelectRobot(robot) {
    setSelectedRobot(robot);
  }

  function handleContinue() {
    if (selectedRobot) {
      localStorage.setItem("selectedRobot", JSON.stringify(selectedRobot));
      navigate("/dashboard");
    }
  }

  function handleLogout() {
    logout();
    navigate("/login");
  }

  return (
    <div className="robot-select-page">
      <header className="robot-header">
        <h1>Select a Robot</h1>
        <button className="logout-btn" onClick={handleLogout}>
          Logout
        </button>
      </header>

      <div className="robot-layout">
        <div className="left-column">
          <div className="robot-grid">
            {loading ? (
              <p style={{ padding: "20px", color: "#999" }}>
                Loading robots...
              </p>
            ) : robots.length === 0 ? (
              <p style={{ padding: "20px", color: "#999" }}>
                No robots found. Add robots via the API.
              </p>
            ) : (
              robots.map((robot) => (
                <div
                  key={robot.id}
                  className={`robot-card ${
                    selectedRobot?.id === robot.id ? "selected" : ""
                  }`}
                  onClick={() => handleSelectRobot(robot)}
                >
                  <div className="robot-icon" role="img" aria-label="Robot">
                    {robot.icon}
                  </div>
                  <h3>{robot.name}</h3>
                  <p className="robot-location">{robot.location}</p>
                  <span
                    className={`robot-status ${robot.status.toLowerCase()}`}
                  >
                    {robot.status}
                  </span>
                </div>
              ))
            )}
          </div>

          <div className="robot-actions-left">
            <button
              className="continue-btn"
              onClick={handleContinue}
              disabled={!selectedRobot}
            >
              Continue to Dashboard
            </button>
          </div>
        </div>

        <div className="robot-map-wrapper">
          <RobotFleetMap
            robots={robots}
            selectedRobot={selectedRobot}
            onSelectRobot={handleSelectRobot}
          />
        </div>
      </div>
    </div>
  );
}
