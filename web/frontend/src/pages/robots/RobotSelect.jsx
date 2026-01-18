import { useState } from "react";
import { useNavigate } from "react-router-dom";
import "./RobotSelect.css";
import RobotFleetMap from "../../services/RobotFleetMap"; // adjust path if needed

// Mock robot data (with GPS)
const MOCK_ROBOTS = [
  {
    id: 1,
    name: "Robot Alpha",
    status: "Active",
    icon: "🤖",
    location: "Garden A",
    lat: 33.6758,
    lng: 35.4655,
  },
  {
    id: 2,
    name: "Robot Beta",
    status: "Active",
    icon: "🤖",
    location: "Garden B",
    lat: 33.6761,
    lng: 35.4660,
  },
  {
    id: 3,
    name: "Robot Gamma",
    status: "Idle",
    icon: "🤖",
    location: "Garden C",
    lat: 33.6754,
    lng: 35.4663,
  },
  {
    id: 4,
    name: "Robot Delta",
    status: "Active",
    icon: "🤖",
    location: "Garden D",
    lat: 33.6759,
    lng: 35.4650,
  },
];

export default function RobotSelect() {
  const [selectedRobot, setSelectedRobot] = useState(null);
  const navigate = useNavigate();

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
    localStorage.removeItem("gg_token");
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

      {/* Cards + Map layout */}
      <div className="robot-layout">
        {/* LEFT: Cards + Continue button */}
        <div className="left-column">
          <div className="robot-grid">
            {MOCK_ROBOTS.map((robot) => (
              <div
                key={robot.id}
                className={`robot-card ${
                  selectedRobot?.id === robot.id ? "selected" : ""
                }`}
                onClick={() => handleSelectRobot(robot)}
              >
                <div className="robot-icon">{robot.icon}</div>
                <h3>{robot.name}</h3>
                <p className="robot-location">{robot.location}</p>
                <span className={`robot-status ${robot.status.toLowerCase()}`}>
                  {robot.status}
                </span>
              </div>
            ))}
          </div>

          {/* Continue button placed under cards (left column) */}
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

        {/* RIGHT: Map */}
        <div className="robot-map-wrapper">
          <RobotFleetMap
            robots={MOCK_ROBOTS}
            selectedRobot={selectedRobot}
            onSelectRobot={handleSelectRobot}
          />
        </div>
      </div>
    </div>
  );
}
