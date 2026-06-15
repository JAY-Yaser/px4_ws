#!/bin/bash
# ============================================================================
# launch_cuadc2_cam_fast.sh — Fast PX4 + camera on CUADC_UAV02 (ROS2 bridge)
#
# Lighter than launch_cuadc2_cam.sh:
#   - Gazebo GUI runs without embedded ImageDisplay (saves GPU)
#   - Camera bridged to ROS2 /downward_camera for YOLO / external use
#   - rqt_image_view can be launched separately with --cam flag
#
# Usage:
#   bash launch/launch_cuadc2_cam_fast.sh              # normal (GUI, no camera panel)
#   bash launch/launch_cuadc2_cam_fast.sh --headless   # no Gazebo GUI at all
#   bash launch/launch_cuadc2_cam_fast.sh --cam        # auto-launch rqt viewer
#
# After launch, YOLO can read:  /downward_camera
# ============================================================================
set -e

HEADLESS=false
AUTO_CAM=false
for arg in "$@"; do
    case $arg in
        --headless) HEADLESS=true ;;
        --cam)      AUTO_CAM=true ;;
    esac
done

# ---- Paths ----------------------------------------------------------------
PX4_DIR="/home/jay/PX4-Autopilot"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

WS="/home/jay/px4_ws"
[ -L "$WS" ] || ln -sf "/home/jay/px4 _ws" "$WS"

WORLD_FILE="$WS/gazebo_models/worlds/CUADC_UAV02.sdf"
WNAME="CUADC_UAV02"
LOCAL_MODELS="$WS/gazebo_models/models"

# ---- PX4 Gazebo resource paths --------------------------------------------
export PX4_GZ_MODELS="$LOCAL_MODELS"
export PX4_GZ_PLUGINS="$PX4_BUILD_DIR/src/modules/simulation/gz_plugins"
export PX4_GZ_SERVER_CONFIG="$PX4_DIR/src/modules/simulation/gz_bridge/server.config"

export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$LOCAL_MODELS:$PX4_DIR/Tools/simulation/gz/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$PX4_GZ_PLUGINS"
export GZ_SIM_SERVER_CONFIG_PATH="$PX4_GZ_SERVER_CONFIG"

# ---- ROS2 environment (isolate from conda Python) -------------------------
# ROS2 Humble needs system Python 3.10; conda Python breaks imports.
# Temporarily remove conda from PATH for ROS2 commands.
for p in /opt/ros/humble/bin /usr/bin; do
    case ":$PATH:" in *":$p:"*) ;; *) export PATH="$p:$PATH" ;; esac
done
unset PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV 2>/dev/null || true

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# ---- Clean up -------------------------------------------------------------
echo "[$(date '+%H:%M:%S')] Stopping previous instances..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ros_gz_image" 2>/dev/null || true
sleep 1

# ---- Gazebo server --------------------------------------------------------
echo "[$(date '+%H:%M:%S')] Starting Gazebo server (CUADC_UAV02)..."
gz sim -r -s "$WORLD_FILE" &
GZ_SERVER_PID=$!
sleep 3

if ! kill -0 "$GZ_SERVER_PID" 2>/dev/null; then
    echo "ERROR: Gazebo server failed to start"
    exit 1
fi

# ---- Gazebo GUI (optional) ------------------------------------------------
if $HEADLESS; then
    echo "[$(date '+%H:%M:%S')] Headless mode — no Gazebo GUI"
else
    echo "[$(date '+%H:%M:%S')] Starting Gazebo GUI..."
    gz sim -g &
    GZ_GUI_PID=$!
    sleep 2
fi

# ---- PX4 ------------------------------------------------------------------
echo "[$(date '+%H:%M:%S')] Launching PX4 X500..."
cd "$PX4_DIR"

PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD="$WNAME" \
PX4_GZ_MODEL_POSE="0,0,0,0,0,1.5708" \
make px4_sitl gz_x500 &
PX4_PID=$!
sleep 5

# ---- ROS2 camera bridge (for YOLO / external use) -------------------------
GZ_TOPIC="/world/$WNAME/model/x500_0/link/base_link/sensor/downward_camera/image"
ROS_TOPIC="/downward_camera"

echo ""
echo "  === ROS2 Camera Bridge ==="
echo "  Gz  topic: $GZ_TOPIC"
echo "  ROS topic: $ROS_TOPIC"

if command -v ros2 &> /dev/null; then
    ros2 run ros_gz_image image_bridge "$GZ_TOPIC" "$ROS_TOPIC" &
    BRIDGE_PID=$!
    echo "  Bridge PID: $BRIDGE_PID"
    echo ""
    echo "  YOLO / Python:"
    echo "    cv2.VideoCapture('/downward_camera')  # via ROS2 + cv_bridge"
    echo ""
else
    echo "  WARNING: ros2 not found, bridge skipped"
fi

# ---- Optional rqt viewer --------------------------------------------------
if $AUTO_CAM && command -v ros2 &> /dev/null; then
    sleep 2
    ros2 run rqt_image_view rqt_image_view "$ROS_TOPIC" &
    CAM_VIEWER_PID=$!
fi

# ---- Wait for PX4 ---------------------------------------------------------
wait "$PX4_PID" 2>/dev/null || true

# ---- Cleanup --------------------------------------------------------------
echo "[$(date '+%H:%M:%S')] Shutting down..."
[ -n "$CAM_VIEWER_PID" ] && kill "$CAM_VIEWER_PID" 2>/dev/null || true
[ -n "$BRIDGE_PID" ] && kill "$BRIDGE_PID" 2>/dev/null || true
[ -n "$GZ_GUI_PID" ] && kill "$GZ_GUI_PID" 2>/dev/null || true
kill "$GZ_SERVER_PID" 2>/dev/null || true
wait 2>/dev/null || true
echo "[$(date '+%H:%M:%S')] Done."
