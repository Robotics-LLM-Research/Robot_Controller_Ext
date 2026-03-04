# spot_ext_python — Developer Guide

This README is for **developers** who want to understand or modify the extension code. For setup and API usage, see the root [README.md](../../README.md) and [docs/README.md](../docs/README.md).

## Module Overview

This Python module is loaded by Isaac Sim when the extension is enabled. It spawns Spot and Crazyflie, starts HTTP APIs, and drives both robots each physics step.

## File Roles

| File | Purpose |
|------|---------|
| `extension.py` | Extension entry point. `on_startup` creates runtimes and API servers; subscribes to timeline and PhysX step events. Spawns Spot via `SpotFlatTerrainPolicy`, positions drone, attaches runtimes on first Play. |
| `api_server.py` | FastAPI apps for Spot (8001) and Drone (8002). Endpoints enqueue commands; `get_sensors` callback returns camera/IMU data. Runs in daemon threads. |
| `spot_control.py` | `SpotRuntime` and `MotionController`. Interprets queued commands (cmd_vel, move, rotate, stop), applies Spot policy actions, reads sensors. |
| `drone_control.py` | `DroneRuntime` for Crazyflie. Handles 3D velocity, move/rotate/altitude, camera look. Drives drone via PhysX. |
| `sensing.py` | `SensorSuite` — captures camera frames and IMU data from USD prims. |
| `utils.py` | `log()` and helpers (e.g. angle wrapping). |
| `global_variables.py` | Extension metadata (title, description) from config. |

## Data Flow

1. **Startup**: Extension starts Spot and Drone API servers; runtimes are created but not yet attached.
2. **First Play**: PhysX ready → spawn Spot, position drone → attach runtimes → subscribe to physics step.
3. **Each physics step**: `_on_world_physics_step` → `spot_runtime.step()` and `drone_runtime.step()`.
4. **HTTP requests**: API endpoints put commands into queues; runtimes consume them in `step()`.

## Extending

- **New endpoints**: Add routes in `api_server.py` and corresponding command handling in `spot_control.py` or `drone_control.py`.
- **New sensors**: Extend `SensorSuite` in `sensing.py` and wire `get_sensors` in the API.
