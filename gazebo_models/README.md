# Gazebo Models

Place Gazebo scene models (`.world`, `.sdf`, model directories) here.

## Directory Structure

```
gazebo_models/
├── worlds/          # .world files - Gazebo scene definitions
├── models/          # Custom .sdf model directories
└── README.md
```

## Worlds

### indoor_20x20 — 20m×20m roofless indoor room

- **Size**: 20m × 20m, walls 3m tall, no roof
- **Gazebo Harmonic 8.x** compatible (SDF 1.11)
- Contains: floor, 4 walls, 2 pillars, low block, tall box, table
- Drone spawn marker at room center (0, 0, 0.3)

**Run standalone:**
```bash
gz sim worlds/indoor_20x20.sdf
```

**Run with PX4 (X500 drone):**
```bash
export PX4_GZ_WORLD=$(pwd)/worlds/indoor_20x20.sdf
make px4_sitl gz_x500
```

## Usage

In PX4 Gazebo simulation, custom models in this directory can be referenced by
adding the path to `GAZEBO_MODEL_PATH`:

```bash
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:$(pwd)/models"
```
