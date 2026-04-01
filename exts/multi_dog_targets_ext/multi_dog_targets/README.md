# Multi Dogs vs Targets

This extension runs a multi-dog Spot scenario in Isaac Sim. It starts five Spot runtimes and a task target, then exposes a local HTTP API for each dog plus a task API for querying and resetting the scenario.

## Load and run

1. Add `root/exts/multi_dog_targets_ext` to Isaac Sim's **Extension Search Paths**.
2. Enable **Multi Dogs vs Targets** from **Window -> Extensions**.
3. Open a compatible world that contains:
   - `/World/dog1` through `/World/dog5`
   - each dog's `body`
   - each dog camera prim (`dog1_cam` through `dog5_cam`)
   - the task target at `/World/Environment/Target`
4. Press **Play** in Isaac Sim to spawn the runtime objects and start the APIs.

## Local APIs

All APIs bind to host `127.0.0.1`.

| Service | Base URL | Purpose |
|------|---------|---------|
| Task API | `http://127.0.0.1:8001` | Query target pose and reset the scenario |
| Dog 1 API | `http://127.0.0.1:8002` | Dog 1 motion, pose, status, and sensors |
| Dog 2 API | `http://127.0.0.1:8003` | Dog 2 motion, pose, status, and sensors |
| Dog 3 API | `http://127.0.0.1:8004` | Dog 3 motion, pose, status, and sensors |
| Dog 4 API | `http://127.0.0.1:8005` | Dog 4 motion, pose, status, and sensors |
| Dog 5 API | `http://127.0.0.1:8006` | Dog 5 motion, pose, status, and sensors |

## Main endpoints

### Dog APIs

Each dog API exposes the standard Spot endpoints:

- `GET /ping`
- `GET /status`
- `GET /pose`
- `POST /cmd_vel`
- `POST /stop`
- `POST /move`
- `POST /rotate`
- `GET /sensors`
- `GET /frame`

### Task API

- `GET /ping`
- `GET /target`
- `POST /reset`

## Developer notes

This Python module is loaded by Isaac Sim when the extension is enabled. It creates one `SpotRuntime` and one API server per dog on startup, attaches all dogs on first play, and advances them each physics step.

| File | Purpose |
|------|---------|
| `extension.py` | Extension entry point. Defines dog paths, API host and ports, startup logic, and physics-step callbacks. |
| `sim_core/api_server.py` | FastAPI apps for Spot and Task services. |
| `sim_core/spot_control.py` | `SpotRuntime` and motion command handling for each dog. |
| `sim_core/task_control.py` | Task target state and reset logic. |
| `sim_core/sensing.py` | Sensor capture helpers for camera and IMU data. |
