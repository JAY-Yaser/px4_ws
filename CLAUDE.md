# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

PX4 SITL + Gazebo Harmonic drone simulation workspace. Contains custom Gazebo worlds (SDF 1.11), launch scripts, and modified drone models. All simulation assets are self-contained — **never modify files inside `/home/jay/PX4-Autopilot/`**.

- **FC**: PX4 v1.17+ (SITL), built at `/home/jay/PX4-Autopilot`
- **Sim**: Gazebo Harmonic 8.11 (`gz-sim8`) on Ubuntu 22.04
- **Airframe**: X500 quadrotor (`make px4_sitl gz_x500`)
- **ROS2**: Humble, `/opt/ros/humble` (Python 3.10; conda `px4` env is compatible)

## Critical path constraint

The workspace path contains a space (`/home/jay/px4 _ws`). Gazebo `file://` URIs break with spaces. Use `/home/jay/px4_ws` (symlink) for all paths that go into `PX4_GZ_MODELS`, `GZ_SIM_RESOURCE_PATH`, or SDF `<mesh><uri>`.

## Launch scripts (`launch/`)

All scripts use **standalone mode** (`PX4_GZ_STANDALONE=1`): they start Gazebo server + GUI independently, then launch PX4 which connects to the already-running world. This avoids touching PX4 internal files.

| Script | World | Model |
|--------|-------|-------|
| `launch_indoor.sh` | indoor_20x20 | x500 |
| `launch_cuadc.sh` | CUADC_UAV01 | x500 |
| `launch_cuadc2.sh` | CUADC_UAV02 | x500 |
| `launch_cuadc2_cam.sh` | CUADC_UAV02 | x500 (local copy with downward camera) |

Key env vars set by launch scripts:
- `PX4_GZ_MODELS` — points to local `gazebo_models/models/` so PX4 finds our drop-in x500
- `GZ_SIM_RESOURCE_PATH` — includes both local models and PX4 models for `model://` URI resolution
- `PX4_GZ_WORLD` — world name (must match `<world name="...">` in SDF)
- `PX4_GZ_MODEL_POSE="0,0,0,0,0,1.5708"` — drone initial yaw (π/2 = faces +Y)

## World SDF requirements

Every world `.sdf` must include (copied from PX4 `default.sdf` pattern):

```xml
<gravity>0 0 -9.8</gravity>
<magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
<atmosphere type="adiabatic"/>
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <latitude_deg>47.397971057728974</latitude_deg>
  <longitude_deg>8.546163739800146</longitude_deg>
  <elevation>0</elevation>
</spherical_coordinates>
```

And these world-level plugins: `gz-sim-physics-system`, `gz-sim-user-commands-system`, `gz-sim-sensors-system` (ogre2), `gz-sim-scene-broadcaster-system`, `gz-sim-contact-system`, `gz-sim-imu-system`, `gz-sim-magnetometer-system`, `gz-sim-navsat-system`, `gz-sim-air-pressure-system`, `gz-sim-air-speed-system`, `gz-sim-apply-link-wrench-system`, `gz-sim-wind-system`.

Without these, PX4 sensors (barometer, magnetometer, GPS) will fail preflight checks.

## Custom x500 model (`gazebo_models/models/x500{,_base}/`)

Drop-in replacement for PX4's x500. `PX4_GZ_MODELS` points here first, so PX4 uses our copy. The `x500_base/model.sdf` adds a downward RGB camera sensor (640×480, 30Hz, topic `downward_camera`).

To modify the camera: edit `gazebo_models/models/x500_base/model.sdf` (the `<sensor name="downward_camera">` block).

## Hollow cylinder meshes

Tube obstacles in CUADC_UAV02 use `.obj` mesh files in `gazebo_models/models/tube_*.obj`. Generated via Python with outer wall + inner wall + sealed bottom. Collision remains solid cylinder for physics; visual uses the mesh.

## Python environment

PX4 builds with conda env `px4` (Python 3.10.20). ROS2 Python tools need system Python 3.10. The conda `px4` environment isolates site-packages — system PyQt5 is invisible. For `rqt_image_view`, either use `conda activate px4` with conda-installed PyQt, or deactivate conda and use system Python.

## Viewing camera feed

1. **Gazebo GUI** (simplest): After drone spawns, open "Downward Camera" docked panel, use topic picker to select camera topic.
2. **ROS2 bridge**: `ros2 run ros_gz_image image_bridge <gz_topic> /downward_camera` then `ros2 run rqt_image_view rqt_image_view /downward_camera`.
