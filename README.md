# Robot Controller Extension

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Robot Controller Extension is a lightweight Isaac Sim extension that exposes simulated robots through local HTTP REST APIs.

It discovers supported robot prims in the active Isaac Sim stage, attaches runtime controllers when the simulation starts, and serves per-robot APIs for motion control, pose/status queries, camera/IMU access, and task-level reset/target operations.

The extension is intended as a simple execution layer for external agents, scripts, notebooks, and LLM-based robotics experiments that need to control Isaac Sim without depending on ROS, MCP, or a custom simulator loop.

|||
|---|---|
|![](docs/assets/spot-targets.gif)|![](docs/assets/spot-house.gif)|

## Features

- **Ground robot control**: Velocity, move/rotate, pose/status, camera/IMU when available
- **Drone control**: 3D velocity, movement/altitude/rotation/look commands, camera/sensors when available
- **REST API**: FastAPI-based local endpoints
- **Per-robot APIs**: One API service per discovered robot
- **Task API**: Target query and scenario reset
- **Interactive docs**: Swagger UI at `/docs`

## Intended use cases

- LLM or agentic robot-control experiments
- Scripted Isaac Sim task execution
- Benchmark environments that need reset/target APIs
- Lightweight robot-control clients outside Isaac Sim
- Quick control/sensing tests without setting up ROS
- Multi-robot experiments with one local API per robot

## Requirements

- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) (2023.1+ recommended)
- Isaac Sim Python dependency: `psutil`

This extension runs inside Isaac Sim's bundled Python environment, not a repo-local virtual environment. Install runtime Python dependencies with Isaac Sim's `python.sh`:

```bash
cd ~/path/to/isaacsim 
./python.sh -m pip install psutil
```

## Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/Robotics-LLM-Research/Robot_Controller_Ext.git
cd Robot_Controller_Ext
```

> **Tip:** Python files in this repo start with `# pyright: reportMissingImports=false` because Isaac Sim provides `omni`, `pxr`, and related packages at runtime. There is no virtual environment in the repo, so editors may otherwise show unresolved-import warnings outside Isaac Sim.

### 2. Open Isaac Sim

Navigate to your Isaac Sim installation and launch it:

```bash
cd ~/path/to/isaacsim/
./isaac-sim.sh
```

### 3. Load the world

1. Go to **File → Open**
2. Navigate to the repo's `environments` folder
3. Choose one of the world files:
   - `environments/spot_drone_world.usd`
   - `environments/spot_target_world.usd`

### 4. Activate the extension

1. In Isaac Sim, go to **Window → Extensions**
2. Click the **three bars (≡)** menu in the Extensions panel
3. Select **Settings**
4. Under "Extension Search Paths", click **Add** and enter the path to `Robot_Controller_ext`, for example `root/Robot_Controller_ext`
5. Press **Enter** to confirm
6. In the list, find **Third Party** and locate **Robot Controller**
7. **Enable** the extension by toggling it on

### 5. Run the simulation

Press **Play** in Isaac Sim to attach to discovered robots and start the HTTP APIs.

For endpoint parameters and Swagger links, see [`docs/README.md`](docs/README.md).

## Supported robot prims

The extension discovers robot prims directly under the stage root when Play starts. Only these name patterns are recognized (case-sensitive):

| Name pattern | Robot type | API |
|--------------|------------|-----|
| `Spot`, `Spot-1`, `Spot-2`, … | Ground robot (quadruped) | Ground robot API |
| `Drone`, `Drone-1`, `Drone-2`, … | Aerial drone | Drone API |

Each discovered robot must have a `body` prim under its root. Optional sensor prims under `body` are `FrontCam` (camera) and `Sensors` (IMU).

Bundled example worlds in `environments/` follow these conventions. Custom stages can use compatible USD assets as long as prim names and layout match.

API ports follow sibling order under the stage root: first robot → `8002`, second → `8003`, and so on.

## Environment setup

Custom worlds should follow these conventions so discovery and control work without code changes.

**Stage root** — Required. Resolved automatically, in order: stage `defaultPrim`, parent of the first `PhysicsScene`, then `/World` or `/Root`. If resolution fails, the extension logs an error and does not start. Set `defaultPrim` on custom stages when possible (e.g. `defaultPrim = "World"`).

**Robot placement** — Robot prims must be **direct children** of the stage root, not nested deeper. See [Supported robot prims](#supported-robot-prims) for recognized names and port order.

**Prim layout** (under each robot root):

| Robot | Required paths | Optional sensor paths |
|-------|----------------|------------------------|
| Ground | `{root}/body` | `{root}/body/FrontCam`, `{root}/body/Sensors` |
| Drone | `{root}/body` | `{root}/body/FrontCam`, `{root}/body/Sensors` |

**Camera and sensors** — Paths are hardcoded. For `/sensors` and `/frame` to work, name prims exactly as follows under each robot's `body` prim:

- **`FrontCam`** — front-facing `UsdGeom.Camera` (RGB + depth)
- **`Sensors`** — IMU sensor prim

If either prim is missing or invalid, the extension logs a warning in the Isaac Sim terminal and skips attaching that sensor. Motion control still works without them.

**Task target** (optional) — `{stage_root}/Environment/Target` for `GET /target` and reset.

**Workflow** — Open the world in Isaac Sim **before** enabling the extension so robots are present at startup. Press **Play** to attach controllers and start robot APIs.

Bundled examples: `environments/spot_drone_world.usd`, `environments/spot_target_world.usd`.

## API overview

All services bind to `127.0.0.1`.

- Task API: `8001`
- First discovered robot: `8002`
- Second discovered robot: `8003`
- Additional robots continue incrementing ports

Interactive Swagger docs: `http://127.0.0.1:<port>/docs`

For detailed endpoint usage, see [`docs/README.md`](docs/README.md).

## Developer notes

The extension is organized as a thin Isaac Sim runtime layer plus separate API/server, robot runtime, task, and sensing modules so the external HTTP interface stays separate from Isaac Sim-specific control logic.

The `robot_controller` Python module is loaded by Isaac Sim when the extension is enabled. It creates the runtimes on startup, attaches them on first play, and advances them each physics step.

| File | Purpose |
|------|---------|
| `Robot_Controller_ext/robot_controller/extension.py` | Extension entry point. Defines robot paths, API host and ports, startup logic, and physics-step callbacks. |
| `Robot_Controller_ext/robot_controller/api_server.py` | FastAPI apps for ground robot, Drone, and task services. |
| `Robot_Controller_ext/robot_controller/spot_control.py` | `SpotRuntime` and motion command handling for the ground robot. |
| `Robot_Controller_ext/robot_controller/drone_control.py` | `DroneRuntime` and command handling for the Drone. |
| `Robot_Controller_ext/robot_controller/task_control.py` | Task target state and reset logic. |
| `Robot_Controller_ext/robot_controller/sensing.py` | Sensor capture helpers for camera and IMU data. |
| `Robot_Controller_ext/robot_controller/utils.py` | Logging and USD pose helpers. |

## Project structure

```
Robot_Controller_Ext/
├── Robot_Controller_ext/   # Isaac Sim extension (add this path in Extension Search Paths)
│   ├── config/             # extension.toml
│   └── robot_controller/   # Extension Python module
├── environments/           # World and simulation assets
├── agent/                  # External agent clients (HTTP)
├── tests/                  # Client-side test scripts
├── docs/                   # API usage reference
└── README.md               # Repo overview and setup
```
