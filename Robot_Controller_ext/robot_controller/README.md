# Robot Controller

This extension runs Spot and Crazyflie robot scenarios in Isaac Sim. It spawns the robots, exposes local HTTP APIs for control and sensing, and provides a task API for querying and resetting the scenario target.

## Load and run

1. Add `root/Robot_Controller_ext` to Isaac Sim's **Extension Search Paths**.
2. Enable **Robot Controller** from **Window → Extensions**.
3. Open a compatible world from the repo `environments` folder:
   - `environments/spot_drone_world.usd`
   - `environments/spot_target_world.usd`
4. Press **Play** in Isaac Sim to spawn the runtime objects and start the APIs.

## Local APIs

All APIs bind to host `127.0.0.1`.

| Service | Base URL | Purpose |
|------|---------|---------|
| Spot API | `http://127.0.0.1:8001` | Spot motion, pose, status, sensors, and camera frame |
| Drone API | `http://127.0.0.1:8002` | Drone motion, status, sensors, and camera frame |
| Task API | `http://127.0.0.1:8003` | Query target pose and reset the scenario |

## Main endpoints

### Spot API

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

This Python module is loaded by Isaac Sim when the extension is enabled. It creates the runtimes on startup, attaches them on first play, and advances them each physics step.

| File | Purpose |
|------|---------|
| `extension.py` | Extension entry point. Defines robot paths, API host and ports, startup logic, and physics-step callbacks. |
| `api_server.py` | FastAPI apps for Spot, Drone, and Task services. |
| `spot_control.py` | `SpotRuntime` and motion command handling for Spot. |
| `drone_control.py` | `DroneRuntime` and command handling for the Crazyflie drone. |
| `task_control.py` | Task target state and reset logic. |
| `sensing.py` | Sensor capture helpers for camera and IMU data. |
| `utils.py` | Logging and USD pose helpers. |
