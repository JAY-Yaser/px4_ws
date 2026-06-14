#!/bin/bash
# ============================================================================
# launch_cuadc2_cam.sh — PX4 X500 + downward RGB camera on CUADC_UAV02 track
#
# Same as launch_cuadc2.sh but spawns a modified X500 with a downward-facing
# RGB camera (640×480, 30Hz).  Camera images are bridged to ROS2.
#
# Prerequisites:
#   ROS2 Humble + ros_gz_bridge installed
#   source /opt/ros/humble/setup.bash
#
# Usage:  bash launch/launch_cuadc2_cam.sh
# ============================================================================
set -e

# ---- Paths ----------------------------------------------------------------
PX4_DIR="/home/jay/PX4-Autopilot"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

# Use symlink path WITHOUT space (px4_ws) for file:// URI safety
WS="/home/jay/px4_ws"
[ -L "$WS" ] || ln -sf "/home/jay/px4 _ws" "$WS"

WORLD_FILE="$WS/gazebo_models/worlds/CUADC_UAV02.sdf"
WNAME="CUADC_UAV02"
LOCAL_MODELS="$WS/gazebo_models/models"

# ---- PX4 Gazebo resource paths --------------------------------------------
# Override PX4_GZ_MODELS to use our local model (x500_cam with downward camera)
export PX4_GZ_MODELS="$LOCAL_MODELS"
export PX4_GZ_PLUGINS="$PX4_BUILD_DIR/src/modules/simulation/gz_plugins"
export PX4_GZ_SERVER_CONFIG="$PX4_DIR/src/modules/simulation/gz_bridge/server.config"

# Gazebo resource path: our models + PX4 models + PX4 plugins
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$LOCAL_MODELS:$PX4_DIR/Tools/simulation/gz/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$PX4_GZ_PLUGINS"
export GZ_SIM_SERVER_CONFIG_PATH="$PX4_GZ_SERVER_CONFIG"

# ---- Clean up previous instances ------------------------------------------
echo "[$(date '+%H:%M:%S')] Stopping any previous Gazebo instances..."
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 1

# ---- Start Gazebo server --------------------------------------------------
echo "[$(date '+%H:%M:%S')] Starting Gazebo server with CUADC_UAV02 world..."
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

# ---- Launch PX4 (standalone mode, custom x500_cam model) -------------------
echo "[$(date '+%H:%M:%S')] Launching PX4 X500 with downward camera..."
cd "$PX4_DIR"

PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD="$WNAME" \
PX4_GZ_MODEL_POSE="0,0,0,0,0,1.5708" \
make px4_sitl gz_x500 &
PX4_PID=$!
sleep 5

# ---- Camera info ----------------------------------------------------------
echo ""
echo "  === Downward camera is active ==="
echo "  View in Gazebo GUI: find 'Downward Camera' docked panel"
echo "  Gazebo topic: /world/$WNAME/model/x500_0/link/base_link/sensor/downward_camera/image"
echo ""

# ---- Wait for PX4 ---------------------------------------------------------
wait "$PX4_PID" 2>/dev/null || true

# ---- Cleanup --------------------------------------------------------------
echo "[$(date '+%H:%M:%S')] PX4 exited, stopping..."
kill "$GZ_SERVER_PID" "$GZ_GUI_PID" 2>/dev/null || true
wait "$GZ_SERVER_PID" "$GZ_GUI_PID" 2>/dev/null || true
echo "[$(date '+%H:%M:%S')] Done."
