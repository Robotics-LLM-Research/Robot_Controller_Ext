# Robot Controller — API Usage

This extension adds HTTP REST APIs to Isaac Sim for controlling robots in the simulation. Once the world is loaded and the simulation is playing, you can send commands and read sensors over HTTP.

## What the extension does

- **Ground robot (port 8001)**: Control a ground-based robot—velocity commands (`cmd_vel`), move/rotate by distance/angle, stop. Read pose, status, camera, and IMU data.
- **Drone (port 8002)**: Control the Drone—3D velocity, move forward/lateral, change altitude, rotate, and camera look. Read camera and IMU via `/sensors`.
- **Task (port 8003)**: Query the scenario target pose and reset the experiment.

All APIs run as background servers inside Isaac Sim and process commands each physics step.

## Setup

See the [root README](../README.md) for full setup instructions. In short:

1. Add `Robot_Controller_ext` to Isaac Sim **Extension Search Paths**.
2. Enable **Robot Controller** from **Window → Extensions**.
3. Open a world from `environments/` (for example `spot_drone_world.usd`).
4. Press **Play**.

For custom worlds, camera and IMU prims must be named **`FrontCam`** and **`Sensors`** under each robot's `body` prim. See the [root README environment setup](../README.md#environment-setup) for details.

## Interactive API documentation

Each API exposes interactive docs (Swagger UI) at `/docs`:

| Service | Base URL | Docs URL |
|---------|----------|----------|
| Ground robot | `http://127.0.0.1:8001` | [http://127.0.0.1:8001/docs](http://127.0.0.1:8001/docs) |
| Drone | `http://127.0.0.1:8002` | [http://127.0.0.1:8002/docs](http://127.0.0.1:8002/docs) |
| Task | `http://127.0.0.1:8003` | [http://127.0.0.1:8003/docs](http://127.0.0.1:8003/docs) |

## Quick reference

**Ground robot endpoints**

- `GET /ping` — Health check
- `GET /status` — Robot status
- `GET /pose` — Robot pose (x, y, z, yaw)
- `POST /cmd_vel?vx=&vy=&wz=` — Continuous velocity (m/s, rad/s)
- `POST /stop` — Stop and zero velocity
- `POST /move?meters=` — Move forward/backward
- `POST /rotate?deg=` — Rotate by degrees
- `GET /sensors` — Camera and IMU data
- `GET /frame` — Latest camera frame as base64 JPEG

**Drone endpoints**

- `GET /ping` — Health check
- `GET /status` — Robot status
- `POST /cmd_vel?vx=&vy=&vz=&wz=` — 3D velocity
- `POST /stop` — Stop
- `POST /move_fwd?meters=` — Move forward/backward
- `POST /move_lat?meters=` — Move left/right
- `POST /raise_alt?meters=` — Change altitude
- `POST /rotate?deg=` — Rotate
- `POST /look?x=&y=` — Move on-board camera
- `GET /sensors` — Camera and IMU data
- `GET /frame` — Latest camera frame as base64 JPEG

**Task endpoints**

- `GET /ping` — Health check
- `GET /target` — Target pose in world frame
- `POST /reset` — Reset robots and scenario
