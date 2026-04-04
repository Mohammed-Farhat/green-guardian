#!/bin/bash
# Start RViz2 streamed to the website via noVNC on port 6080.
# Requires: sudo apt install xvfb x11vnc novnc
# Usage: ./start_rviz_novnc.sh

set -e

DISPLAY_NUM=:1
VNC_PORT=5900
NOVNC_PORT=6080
RESOLUTION="1280x720x24"
NOVNC_PATH="/usr/share/novnc"

# ── Dependency check ──────────────────────────────────────────────────────────
for cmd in Xvfb x11vnc websockify; do
  if ! command -v "$cmd" &>/dev/null; then
    echo "ERROR: '$cmd' not found. Run:"
    echo "  sudo apt install xvfb x11vnc novnc"
    exit 1
  fi
done

if [ ! -d "$NOVNC_PATH" ]; then
  echo "ERROR: noVNC web files not found at $NOVNC_PATH"
  echo "  sudo apt install novnc"
  exit 1
fi

# ── Source ROS2 ───────────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
INSTALL_SETUP="$( cd "$(dirname "$0")/ros2_ws" && pwd )/install/setup.bash"
if [ -f "$INSTALL_SETUP" ]; then
  source "$INSTALL_SETUP"
else
  echo "WARNING: ROS2 workspace not built. Run:"
  echo "  cd ros2_ws && colcon build --packages-select robot_description"
fi

# ── Cleanup on exit ───────────────────────────────────────────────────────────
cleanup() {
  echo "Stopping all processes..."
  kill "$XVFB_PID" "$X11VNC_PID" "$NOVNC_PID" "$RVIZ_PID" 2>/dev/null || true
  rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
}
trap cleanup EXIT INT TERM

# ── Start Xvfb ────────────────────────────────────────────────────────────────
echo "Starting virtual display $DISPLAY_NUM ($RESOLUTION)..."
rm -f /tmp/.X1-lock
Xvfb "$DISPLAY_NUM" -screen 0 "$RESOLUTION" &
XVFB_PID=$!
sleep 1

# ── Start x11vnc ─────────────────────────────────────────────────────────────
echo "Starting x11vnc on port $VNC_PORT..."
x11vnc -display "$DISPLAY_NUM" -nopw -listen localhost \
       -rfbport "$VNC_PORT" -forever -shared -quiet &
X11VNC_PID=$!
sleep 1

# ── Start noVNC / websockify ──────────────────────────────────────────────────
echo "Starting noVNC on port $NOVNC_PORT..."
websockify --web "$NOVNC_PATH" "$NOVNC_PORT" "localhost:$VNC_PORT" &
NOVNC_PID=$!
sleep 1

# ── Launch RViz2 ──────────────────────────────────────────────────────────────
echo "Launching RViz2..."
DISPLAY="$DISPLAY_NUM" ros2 launch robot_description view_model.launch.py &
RVIZ_PID=$!

echo ""
echo "============================================"
echo "  noVNC ready at: http://localhost:$NOVNC_PORT/vnc.html"
echo "  Open the website Tracking tab to see RViz2"
echo "  Press Ctrl+C to stop everything"
echo "============================================"
echo ""

wait "$RVIZ_PID"
