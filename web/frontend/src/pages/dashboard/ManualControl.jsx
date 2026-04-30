import { useEffect, useRef, useState } from "react";
import { toast } from "react-toastify";
import { robotApi } from "../../services/api";

function dirToVelocity(dir, linear, angular) {
  switch (dir) {
    case "forward":  return [linear, 0];
    case "backward": return [-linear, 0];
    case "left":     return [0, angular];
    case "right":    return [0, -angular];
    default:         return [0, 0];
  }
}

export default function ManualControl() {
  const [isManual, setIsManual] = useState(false);
  const [linearSpeed, setLinearSpeed] = useState(0.5);
  const [angularSpeed, setAngularSpeed] = useState(0.5);
  const [activeDir, setActiveDir] = useState(null);
  const [emergencyStopped, setEmergencyStopped] = useState(false);
  const [emergencyCountdown, setEmergencyCountdown] = useState(0);

  // Refs so keyboard listener always reads latest values without re-registering
  const isManualRef = useRef(isManual);
  const linearRef = useRef(linearSpeed);
  const angularRef = useRef(angularSpeed);
  const emergencyRef = useRef(emergencyStopped);
  const activeDirRef = useRef(activeDir);
  const intervalRef = useRef(null);

  useEffect(() => { isManualRef.current = isManual; }, [isManual]);
  useEffect(() => { linearRef.current = linearSpeed; }, [linearSpeed]);
  useEffect(() => { angularRef.current = angularSpeed; }, [angularSpeed]);
  useEffect(() => { emergencyRef.current = emergencyStopped; }, [emergencyStopped]);
  useEffect(() => { activeDirRef.current = activeDir; }, [activeDir]);

  function sendVelocity(lin, ang) {
    robotApi.teleop(lin, ang).catch((err) => {
      console.error("Teleop failed:", err.message);
    });
  }

  function startMoving(dir) {
    if (!isManualRef.current || emergencyRef.current) return;
    stopMoving(false);
    const [lin, ang] = dirToVelocity(dir, linearRef.current, angularRef.current);
    sendVelocity(lin, ang);
    setActiveDir(dir);
    intervalRef.current = setInterval(() => {
      const [l, a] = dirToVelocity(dir, linearRef.current, angularRef.current);
      sendVelocity(l, a);
    }, 200);
  }

  function stopMoving(sendStop = true) {
    clearInterval(intervalRef.current);
    intervalRef.current = null;
    setActiveDir(null);
    if (sendStop) {
      robotApi.stop().catch(() => {});
    }
  }

  function handleEmergencyStop() {
    stopMoving(true);
    setEmergencyStopped(true);
    setEmergencyCountdown(3);
    toast.error("Emergency stop triggered!");

    let count = 3;
    const timer = setInterval(() => {
      count -= 1;
      setEmergencyCountdown(count);
      if (count <= 0) {
        clearInterval(timer);
        setEmergencyStopped(false);
        setEmergencyCountdown(0);
      }
    }, 1000);
  }

  function handleModeToggle() {
    if (isManual) {
      stopMoving(true);
    }
    setIsManual((v) => !v);
  }

  // Keyboard control
  useEffect(() => {
    const KEY_MAP = {
      ArrowUp: "forward", w: "forward", W: "forward",
      ArrowDown: "backward", s: "backward", S: "backward",
      ArrowLeft: "left", a: "left", A: "left",
      ArrowRight: "right", d: "right", D: "right",
    };

    const handleKeyDown = (e) => {
      if (!isManualRef.current || emergencyRef.current) return;
      if (e.repeat) return;
      const dir = KEY_MAP[e.key];
      if (dir) {
        e.preventDefault();
        startMoving(dir);
      }
      if (e.key === " ") {
        e.preventDefault();
        handleEmergencyStop();
      }
    };

    const handleKeyUp = (e) => {
      const dir = KEY_MAP[e.key];
      if (dir && activeDirRef.current === dir) {
        stopMoving(true);
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      clearInterval(intervalRef.current);
    };
  }, []);

  // Cleanup on unmount — send stop
  useEffect(() => {
    return () => {
      clearInterval(intervalRef.current);
      robotApi.stop().catch(() => {});
    };
  }, []);

  const btnProps = (dir) => ({
    className: `dpad-btn${activeDir === dir ? " active" : ""}${!isManual || emergencyStopped ? " dpad-btn-disabled" : ""}`,
    onMouseDown: (e) => { e.preventDefault(); startMoving(dir); },
    onMouseUp: () => stopMoving(true),
    onMouseLeave: () => { if (activeDirRef.current === dir) stopMoving(true); },
    onTouchStart: (e) => { e.preventDefault(); startMoving(dir); },
    onTouchEnd: (e) => { e.preventDefault(); stopMoving(true); },
    disabled: !isManual || emergencyStopped,
  });

  return (
    <div className="control-layout">
      {/* Left panel: D-pad + sliders + E-stop */}
      <div className="control-panel card">
        <p className="card-title">
          <span className={`mode-dot ${isManual ? "mode-dot-manual" : "mode-dot-auto"}`} />
          {isManual ? "Manual Control Active" : "Autonomous Mode"}
        </p>

        <button
          className={`mode-toggle-btn ${isManual ? "mode-toggle-manual" : "mode-toggle-auto"}`}
          onClick={handleModeToggle}
        >
          {isManual ? "Switch to Autonomous" : "Take Manual Control"}
        </button>

        {/* D-Pad */}
        <div className="dpad-grid">
          <div className="dpad-empty" />
          <button {...btnProps("forward")}>↑</button>
          <div className="dpad-empty" />

          <button {...btnProps("left")}>←</button>
          <button
            className={`dpad-btn dpad-stop${!isManual || emergencyStopped ? " dpad-btn-disabled" : ""}`}
            onMouseDown={(e) => { e.preventDefault(); stopMoving(true); }}
            onTouchStart={(e) => { e.preventDefault(); stopMoving(true); }}
            disabled={!isManual || emergencyStopped}
          >
            ■
          </button>
          <button {...btnProps("right")}>→</button>

          <div className="dpad-empty" />
          <button {...btnProps("backward")}>↓</button>
          <div className="dpad-empty" />
        </div>

        {/* Speed sliders */}
        <div className="speed-controls">
          <div className="speed-row">
            <label className="speed-label">
              Linear <span className="speed-val">{linearSpeed.toFixed(1)} m/s</span>
            </label>
            <input
              type="range" min="0.1" max="1.0" step="0.1"
              value={linearSpeed}
              onChange={(e) => setLinearSpeed(parseFloat(e.target.value))}
              className="speed-slider"
            />
          </div>
          <div className="speed-row">
            <label className="speed-label">
              Angular <span className="speed-val">{angularSpeed.toFixed(1)} rad/s</span>
            </label>
            <input
              type="range" min="0.1" max="1.5" step="0.1"
              value={angularSpeed}
              onChange={(e) => setAngularSpeed(parseFloat(e.target.value))}
              className="speed-slider"
            />
          </div>
        </div>

        {/* Emergency Stop */}
        <button
          className="emergency-stop-btn"
          onClick={handleEmergencyStop}
          disabled={emergencyStopped}
        >
          {emergencyStopped ? `LOCKED (${emergencyCountdown}s)` : "⚠ EMERGENCY STOP"}
        </button>
      </div>

      {/* Right panel: status + cheatsheet */}
      <div className="control-info">
        <div className="card control-status-card">
          <p className="card-title">Status</p>
          <div className={`mode-indicator ${isManual ? "mode-indicator-manual" : "mode-indicator-auto"}`}>
            {isManual ? "MANUAL MODE" : "AUTONOMOUS MODE"}
          </div>
          {emergencyStopped && (
            <div className="emergency-active-badge">
              EMERGENCY STOP ACTIVE — {emergencyCountdown}s
            </div>
          )}
          <div className="status-row">
            <span className="status-label">Linear Speed</span>
            <span className="status-val">{linearSpeed.toFixed(1)} m/s</span>
          </div>
          <div className="status-row">
            <span className="status-label">Angular Speed</span>
            <span className="status-val">{angularSpeed.toFixed(1)} rad/s</span>
          </div>
          <div className="status-row">
            <span className="status-label">Direction</span>
            <span className="status-val">{activeDir || "stopped"}</span>
          </div>
        </div>

        <div className="card control-cheatsheet">
          <p className="card-title">Keyboard Shortcuts</p>
          <div className="cheatsheet-grid">
            <kbd>W</kbd><span>Forward</span>
            <kbd>S</kbd><span>Backward</span>
            <kbd>A</kbd><span>Turn Left</span>
            <kbd>D</kbd><span>Turn Right</span>
            <kbd>↑↓←→</kbd><span>Arrow Keys</span>
            <kbd>Space</kbd><span>Emergency Stop</span>
          </div>
          <p className="cheatsheet-note">
            Hold key to keep moving. Release to stop.
            <br />Switch to Manual Mode first.
          </p>
        </div>
      </div>
    </div>
  );
}
