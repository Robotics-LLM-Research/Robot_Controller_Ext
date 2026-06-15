# Robot Controller Extension

An Isaac Sim extension for controlling simulated robots via HTTP REST API. The extension spawns the robots, exposes local HTTP APIs for control and sensing, and provides a task API for querying and resetting the scenario target.

## Features

- **Ground robot**: Velocity control, move/rotate commands, pose and status, camera and IMU sensors
- **Drone**: Full 3D velocity control, move/rotate/altitude, camera
- **REST API**: FastAPI-based endpoints for programmatic control
- **Interactive docs**: Swagger UI at `/docs` for each API

## Requirements

- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) (2023.1+ recommended)

## Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/Diego5936/Spot_Extension.git
cd Spot_Extension
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

## Environment setup

Custom worlds should follow these conventions so discovery and control work without code changes.

**Stage root** — Required. Resolved automatically, in order: stage `defaultPrim`, parent of the first `PhysicsScene`, then `/World` or `/Root`. If resolution fails, the extension logs an error and does not start. Set `defaultPrim` on custom stages when possible (e.g. `defaultPrim = "World"`).

**Robot placement** — Robot prims must be **direct children** of the stage root, not nested deeper.

**Robot names** — Prim names must match exactly (case-sensitive):

| Name pattern | Type |
|--------------|------|
| `Spot`, `Spot-1`, `Spot-2`, … | Ground robot |
| `Drone`, `Drone-1`, `Drone-2`, … | Drone |

API ports follow **sibling order** under the stage root (see [APIs](#apis)).

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
## APIs

All services bind to host **`127.0.0.1`** and run as background servers inside Isaac Sim. Commands are processed each physics step.

**Port assignment**

- **Task API** always uses the base port **`8001`**.
- Each robot discovered in the stage (in discovery order) gets the next port: **`8002`**, **`8003`**, and so on.
- Discovery matches prims directly under the stage root named `Spot`, `Drone`, `Spot-1`, `Spot-2`, `Drone-1`, etc.

Interactive docs (Swagger UI) for each service: `http://127.0.0.1:<port>/docs`

| Robot kind | Endpoints include |
|------------|-------------------|
| Ground (`Spot`, `Spot-N`) | `/status`, `/pose`, `/cmd_vel`, `/move`, `/rotate`, `/sensors`, `/frame`, … |
| Drone (`Drone`, `Drone-N`) | `/status`, `/cmd_vel`, `/move_fwd`, `/move_lat`, `/raise_alt`, `/look`, `/sensors`, `/frame`, … |
| Task | `/target`, `/reset` |

## Main endpoints

### Ground robot API

- `GET /ping`
- `GET /status`
- `GET /pose`
- `POST /cmd_vel`
- `POST /stop`
- `POST /move`
- `POST /rotate`
- `GET /sensors`
- `GET /frame`

### Drone API

- `GET /ping`
- `GET /status`
- `POST /cmd_vel`
- `POST /stop`
- `POST /move_fwd`
- `POST /move_lat`
- `POST /raise_alt`
- `POST /rotate`
- `POST /look`
- `GET /sensors`
- `GET /frame`

### Task API

- `GET /ping`
- `GET /target`
- `POST /reset`

## Developer notes

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
Spot_Extension/
├── Robot_Controller_ext/   # Isaac Sim extension (add this path in Extension Search Paths)
│   ├── config/               # extension.toml
│   └── robot_controller/     # Extension Python module
├── environments/             # World and simulation assets
├── agent/                    # External agent clients (HTTP)
├── tests/                    # Client-side test scripts
├── docs/                     # API usage reference
└── README.md                 # This file
```
