#!/usr/bin/env bash
set -euo pipefail

export DISPLAY=${DISPLAY:-:1}
export VNC_PORT=${VNC_PORT:-5901}
export NOVNC_PORT=${NOVNC_PORT:-6080}
export VNC_RESOLUTION=${VNC_RESOLUTION:-1600x900}
export VNC_COL_DEPTH=${VNC_COL_DEPTH:-24}
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-root}

mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"

# Start virtual X server
Xvfb "${DISPLAY}" -screen 0 "${VNC_RESOLUTION}x${VNC_COL_DEPTH}" -ac +extension GLX +render -noreset &
XVFB_PID=$!

# Give Xvfb a moment
sleep 2

# Start XFCE session
startxfce4 >/tmp/xfce.log 2>&1 &
XFCE_PID=$!

# Start VNC server attached to the Xvfb display
x11vnc \
  -display "${DISPLAY}" \
  -forever \
  -shared \
  -nopw \
  -listen 0.0.0.0 \
  -xkb \
  -rfbport "${VNC_PORT}" \
  >/tmp/x11vnc.log 2>&1 &
VNC_PID=$!

# Start noVNC/websockify
websockify --web=/usr/share/novnc/ "${NOVNC_PORT}" "localhost:${VNC_PORT}" \
  >/tmp/novnc.log 2>&1 &
NOVNC_PID=$!

echo "Desktop ready:"
echo "  VNC:   ${VNC_PORT}"
echo "  noVNC: ${NOVNC_PORT}"
echo "Open: http://<server-ip>:${NOVNC_PORT}/"

trap 'kill ${NOVNC_PID} ${VNC_PID} ${XFCE_PID} ${XVFB_PID} 2>/dev/null || true' SIGINT SIGTERM

wait -n ${NOVNC_PID} ${VNC_PID} ${XFCE_PID} ${XVFB_PID}