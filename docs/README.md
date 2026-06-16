# Robot Controller for Isaac Sim

This extension runs local HTTP APIs inside Isaac Sim so you can control robots and read sensors while the simulation is playing. When Play starts, it attaches robot runtimes and serves APIs on `127.0.0.1`.

## API services and ports

The extension starts one task API and one API per discovered robot:

- Task API: base port `8001`
- First robot: `8002`
- Second robot: `8003`
- Additional robots continue incrementing ports

Discovery is based on prims directly under the stage root named:

- Ground robot: `Spot`, `Spot-1`, `Spot-2`, ...
- Drone: `Drone`, `Drone-1`, `Drone-2`, ...

Swagger UI is available for each active service at:

- `http://127.0.0.1:<port>/docs`

## Stage requirements

For reliable startup and sensing, use these conventions:

- Stage root should resolve through `defaultPrim` (recommended).
- Robot prims should be direct children of the stage root.
- Each robot should contain a `body` prim.
- Sensor prim names under `body` must match exactly:
  - `FrontCam` for camera endpoints
  - `Sensors` for IMU endpoints
- Optional task target prim: `{stage_root}/Environment/Target`

If a required robot body is missing, that robot is skipped. If `FrontCam` or `Sensors` is missing, motion control still works but related sensor endpoints return unavailable data.

## Endpoint quick reference

Ground robot endpoints:

- `GET /ping`
- `GET /status`
- `GET /pose`
- `POST /cmd_vel?vx=&vy=&wz=`
- `POST /stop`
- `POST /move?meters=`
- `POST /rotate?deg=`
- `GET /sensors`
- `GET /frame`

Drone endpoints:

- `GET /ping`
- `GET /status`
- `POST /cmd_vel?vx=&vy=&vz=&wz=`
- `POST /stop`
- `POST /move_fwd?meters=`
- `POST /move_lat?meters=`
- `POST /raise_alt?meters=`
- `POST /rotate?deg=`
- `POST /look?x=&y=`
- `GET /sensors`
- `GET /frame`

Task endpoints:

- `GET /ping`
- `GET /target`
- `POST /reset`
