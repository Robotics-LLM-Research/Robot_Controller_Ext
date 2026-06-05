# Spot Extension

An Isaac Sim extension for controlling Boston Dynamics Spot and Crazyflie drone robots via HTTP REST API. Control locomotion, read sensors (camera, IMU), and drive the simulation from external applications.

## Features

- **Spot robot**: Velocity control, move/rotate commands, camera and IMU sensors
- **Crazyflie drone**: Full 3D velocity control, move/rotate/altitude, camera
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

### 2. Open Isaac Sim

Navigate to your Isaac Sim installation and launch it:

```bash
cd <isaac_sim_install_path>
./isaac-sim.sh
```

> **Tip**: On Linux, `isaac-sim.sh` is typically in the root of the Isaac Sim installation. On Windows, use the provided launcher or `isaac-sim.bat`.

### 3. Activate the extension

1. In Isaac Sim, go to **Window → Extensions**
2. Click the **three bars (≡)** menu in the Extensions panel
3. Select **Settings**
4. Under "Extension Search Paths", click **Add** and enter the path to `Robot_Controller_ext`, for example `root/Robot_Controller_ext`
5. Press **Enter** to confirm
6. In the list, find **Third Party** and locate **Robot Controller**
7. **Enable** the extension by toggling it on

### 4. Load the world

1. Go to **File → Open**
2. Navigate to the repo's `environments` folder
3. Choose one of the world files:
   - `environments/spot_drone_world.usd`
   - `environments/spot_target_world.usd`

### 5. Run the simulation

Press **Play** in Isaac Sim to spawn the robots and start the HTTP APIs.

For API details and endpoint reference, see `Robot_Controller_ext/robot_controller/README.md` and `docs/README.md`.

## Project structure

```
Spot_Extension/
├── Robot_Controller_ext/   # Isaac Sim extension (add this path in Extension Search Paths)
├── environments/           # World and simulation assets
├── agent/                  # External agent clients (HTTP)
├── tests/                  # Client-side test scripts
├── docs/                   # API documentation
└── README.md               # This file
```
