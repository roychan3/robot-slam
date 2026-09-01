#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/ros2_ws/install/setup.bash

# Keep resource lookup local and deterministic. The world contains its own
# ground plane and sun, so Gazebo never needs to download Fuel models.
export GZ_SIM_RESOURCE_PATH="/app/ros2_ws/install/share:/app/ros2_ws/src/robot_slam/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# Depth cameras use Gazebo's rendering engine even when the GUI is disabled.
# Create a software-backed display only when the caller did not provide one.
if [[ -z "${DISPLAY:-}" ]]; then
  export DISPLAY=:99
  export LIBGL_ALWAYS_SOFTWARE=1
  Xvfb :99 -screen 0 2560x1440x24 -nolisten tcp >/tmp/xvfb.log 2>&1 &
fi

if [[ "${ENABLE_NOVNC:-0}" == "1" ]]; then
  # x11vnc exits immediately if XOpenDisplay fails, and -forever only governs
  # client reconnects, so wait for Xvfb to actually accept connections instead
  # of guessing at a sleep.
  for _ in $(seq 1 50); do
    if xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
  if ! xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
    echo "entrypoint: ${DISPLAY} never became ready; see /tmp/xvfb.log" >&2
  else
    # Browser/host shortcut handling makes Alt+Tab unreliable inside noVNC.
    # Openbox applies class-based rules that keep Gazebo and RViz side by side.
    openbox --sm-disable \
      --config-file /app/ros2_ws/src/robot_slam/config/openbox.xml \
      >/tmp/openbox.log 2>&1 &
    x11vnc -display "${DISPLAY}" -forever -shared -nopw -rfbport 5900 \
      >/tmp/x11vnc.log 2>&1 &
    websockify --web=/usr/share/novnc 6080 localhost:5900 \
      >/tmp/websockify.log 2>&1 &
  fi
fi

# A bind mount keeps host ownership, so the image's chown of /data does not
# survive it. Warn at startup rather than letting save_pointcloud.py fail with
# EACCES after a long drive.
if [[ -d /data && ! -w /data ]]; then
  echo "entrypoint: /data is not writable by uid $(id -u)." >&2
  echo "  The ./data bind mount keeps host ownership. On Linux, re-run with:" >&2
  echo "    HOST_UID=\$(id -u) HOST_GID=\$(id -g) docker compose up" >&2
  echo "  Simulation continues; only saving to /data will fail." >&2
fi

exec "$@"
