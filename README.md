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

Clone to your desired location. For example, to clone into `~/projects`:

```bash
git clone https://github.com/Diego5936/Spot_Extension.git ~/projects/Spot_Extension
cd ~/projects/Spot_Extension
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
4. Under "Extension Search Paths", click **Add** and enter the path to the **root folder** of this repo (the folder containing `config`, `docs`, and `spot_ext_python`)
5. Press **Enter** to confirm
6. In the list, find **Third Party** and locate **SPOT_EXT**
7. **Enable** the extension by toggling it on

### 4. Load the world

1. Go to **File → Open**
2. Navigate to the cloned repo folder
3. Open **`spot_world.usd`**

The world will load with Spot and Crazyflie ready for simulation.

### 5. Run the simulation

Press **Play** in the Isaac Sim timeline. The extension starts HTTP APIs:

- **Spot**: `http://127.0.0.1:8001`
- **Drone**: `http://127.0.0.1:8002`

See [docs/README.md](docs/README.md) for API usage and interactive documentation.

## Project structure

```
Spot_Extension/
├── config/           # Extension configuration
├── docs/             # Extension readme and changelog
├── spot_ext_python/  # Extension Python module
├── spot_world.usd    # Simulation world
└── README.md         # This file
```
