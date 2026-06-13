#!/bin/bash
# ============================================================================
# launch_cuadc.sh — PX4 X500 drone on CUADC_UAV01 race track
#
# Outdoor 300m×300m ground with 8m×65m dark gray runway.
# Orange takeoff pad at origin, white H marker, gate lines at y=32/37/57.
#
# Usage:  bash launch_cuadc.sh
# ============================================================================
set -e

# ---- Paths ----------------------------------------------------------------
PX4_DIR="/home/jay/PX4-Autopilot"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
WORLD_FILE="/home/jay/px4 _ws/gazebo_models/worlds/CUADC_UAV01.sdf"
WNAME="CUADC_UAV01"   # must match <world name="…"> in the .sdf

# ---- PX4 Gazebo resource paths --------------------------------------------
export PX4_GZ_MODELS="$PX4_DIR/Tools/simulation/gz/models"
export PX4_GZ_PLUGINS="$PX4_BUILD_DIR/src/modules/simulation/gz_plugins"
export PX4_GZ_SERVER_CONFIG="$PX4_DIR/src/modules/simulation/gz_bridge/server.config"

export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$PX4_GZ_MODELS"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$PX4_GZ_PLUGINS"
export GZ_SIM_SERVER_CONFIG_PATH="$PX4_GZ_SERVER_CONFIG"

# ---- Clean up previous instances ------------------------------------------
echo "[$(date '+%H:%M:%S')] Stopping any previous Gazebo instances..."
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 1

# ---- Start Gazebo server --------------------------------------------------
echo "[$(date '+%H:%M:%S')] Starting Gazebo server with CUADC_UAV01 world..."
gz sim -r -s "$WORLD_FILE" &
GZ_SERVER_PID=$!
sleep 3

if ! kill -0 "$GZ_SERVER_PID" 2>/dev/null; then
    echo "ERROR: Gazebo server failed to start"
    exit 1
fi

# ---- Start Gazebo GUI -----------------------------------------------------
echo "[$(date '+%H:%M:%S')] Starting Gazebo GUI..."
gz sim -g &
GZ_GUI_PID=$!
sleep 2

# ---- Launch PX4 (standalone mode) -----------------------------------------
echo "[$(date '+%H:%M:%S')] Launching PX4 X500..."
cd "$PX4_DIR"

PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD="$WNAME" \
PX4_GZ_MODEL_POSE="0,0,0,0,0,1.5708" \
make px4_sitl gz_x500

# ---- Cleanup on PX4 exit --------------------------------------------------
echo "[$(date '+%H:%M:%S')] PX4 exited, stopping Gazebo..."
kill "$GZ_SERVER_PID" "$GZ_GUI_PID" 2>/dev/null || true
wait "$GZ_SERVER_PID" "$GZ_GUI_PID" 2>/dev/null || true
echo "[$(date '+%H:%M:%S')] Done."
