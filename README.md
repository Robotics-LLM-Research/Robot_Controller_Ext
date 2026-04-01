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
4. Under "Extension Search Paths", click **Add** and enter the path to the specific extension folder under `exts` for the extension you want to use, for example `root/exts/<extension_folder>`
5. Press **Enter** to confirm
6. In the list, find **Third Party** and locate the extension you added
7. **Enable** the extension by toggling it on

### 4. Load the world

1. Go to **File → Open**
2. Navigate to the repo's `assets` folder
3. Choose one of the world files that exists there for the extension you want to run

The selected world will load with the assets for that scenario ready for simulation.

### 5. Run the selected extension

Each extension can have different controls, APIs, and simulation behavior.

After loading the world, refer to the README inside that extension folder under `exts/<extension_folder>` for the correct run instructions and usage details.

## Project structure

```
Spot_Extension/
├── assets/           # World and simulation assets
├── docs/             # Project documentation
├── exts/             # Isaac Sim extensions
└── README.md         # This file
```
