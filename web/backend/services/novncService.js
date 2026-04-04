const { spawn, execSync } = require("child_process");
const DISPLAY = ":1";
const VNC_PORT = 5900;
const NOVNC_PORT = 6080;
const NOVNC_PATH = "/usr/share/novnc";

let processes = [];

function isAvailable(cmd) {
  try {
    execSync(`which ${cmd}`, { stdio: "ignore" });
    return true;
  } catch {
    return false;
  }
}

function spawnProc(cmd, args, env = {}) {
  const proc = spawn(cmd, args, {
    env: { ...process.env, ...env },
    stdio: "ignore",
    detached: false,
  });
  proc.on("error", (err) => console.error(`[novnc] ${cmd} error:`, err.message));
  processes.push(proc);
  return proc;
}

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

async function start() {
  const required = ["Xvfb", "x11vnc", "websockify"];
  const missing = required.filter((c) => !isAvailable(c));
  if (missing.length > 0) {
    console.warn(`[novnc] Skipping noVNC — missing: ${missing.join(", ")}`);
    console.warn("[novnc] Install with: sudo apt install xvfb x11vnc novnc");
    return;
  }

  try {
    execSync("pkill -f 'Xvfb :1' || true", { stdio: "ignore" });
    execSync("rm -f /tmp/.X1-lock", { stdio: "ignore" });
  } catch {}

  console.log("[novnc] Starting Xvfb...");
  spawnProc("Xvfb", [DISPLAY, "-screen", "0", "1280x720x24"]);
  await sleep(1000);

  console.log(`[novnc] Starting x11vnc on port ${VNC_PORT}...`);
  spawnProc("x11vnc", [
    "-display", DISPLAY,
    "-nopw", "-listen", "localhost",
    "-rfbport", String(VNC_PORT),
    "-forever", "-shared", "-quiet",
  ]);
  await sleep(1000);

  console.log(`[novnc] Starting websockify on port ${NOVNC_PORT}...`);
  spawnProc("websockify", [
    "--web", NOVNC_PATH,
    String(NOVNC_PORT),
    `localhost:${VNC_PORT}`,
  ]);
  await sleep(1000);

  console.log(`[novnc] Ready — http://localhost:${NOVNC_PORT}/vnc.html`);
  console.log(`[novnc] Start RViz2 manually with: DISPLAY=${DISPLAY} ros2 launch robot_description view_model.launch.py`);
}

function stop() {
  processes.forEach((p) => {
    try { p.kill(); } catch {}
  });
  processes = [];
}

process.on("exit", stop);
process.on("SIGINT", () => { stop(); process.exit(); });
process.on("SIGTERM", () => { stop(); process.exit(); });

module.exports = { start };
