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
bash ../launch/launch_indoor.sh
```

### CUADC_UAV01 — 300m×300m outdoor race track

- **Ground**: 300m × 300m flat green plane
- **Track**: 8m × 65m dark gray runway, y: -3 to 62
- **Takeoff pad**: orange circle Ø1m at origin, white "H" marker
- **Gate lines**: white stripes at y=32, y=37, y=57
- **Gazebo Harmonic 8.x** compatible (SDF 1.11)

**Run with PX4 (X500 drone):**
```bash
bash ../launch/launch_cuadc.sh
```

### CUADC_UAV02 — Race track with cylinder obstacles

Same as CUADC_UAV01 plus:
- **Region 1** (y:32~37): 3 white hollow cylinders, H=30cm
  - D=15cm, D=20cm, D=25cm (wall 0.5cm)
- **Region 2** (y:57~62): 5 white hollow cylinders, H=15cm, D=20cm
  - 3 cylinders contain 12×12cm hazard chemical signs (red plate)

**Run with PX4 (X500 drone):**
```bash
bash ../launch/launch_cuadc2.sh
```

## Usage

In PX4 Gazebo simulation, custom models in this directory can be referenced by
adding the path to `GAZEBO_MODEL_PATH`:

```bash
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:$(pwd)/models"
```
