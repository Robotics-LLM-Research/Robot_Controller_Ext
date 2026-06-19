# Robot Controller Extension User Guide

This guide is for users who already see **Robot Controller** inside Isaac Sim's Extensions panel.

The extension starts local HTTP APIs while the simulation is playing. These APIs let external programs control supported robots, read pose/status data, access sensors when available, and call task-level utilities such as target lookup and scenario reset.

## Activate the extension

1. Open an Isaac Sim stage that contains supported robot prims.
2. Go to **Window → Extensions**.
3. Search for **Robot Controller**.
4. Enable the extension.
5. Press **Play**.

The APIs start after Play is pressed. If the extension is enabled but the simulation is not playing, robot runtimes may not be attached yet.

## Supported robot prims

The extension looks for robot prims that are **direct children** of the stage root. Only these names are recognized (case-sensitive):

| Name pattern | Robot type | API |
|--------------|------------|-----|
| `Spot`, `Spot-1`, `Spot-2`, … | Ground robot (quadruped) | Ground robot API |
| `Drone`, `Drone-1`, `Drone-2`, … | Aerial drone | Drone API |

Each robot root must include a `body` prim. For camera and IMU endpoints, also include `FrontCam` and `Sensors` under `body` with those exact names.

If your stage uses different prim names or nesting, the extension will not attach a runtime for that robot.

## Check that the APIs are running

Open these URLs in a browser:

```text
http://127.0.0.1:8001/docs
http://127.0.0.1:8002/docs
```

`8001` is the task API. `8002` is usually the first discovered robot. `8003`, `8004`, and higher ports are used for additional robots.

If `8002` does not open, check that the stage contains a supported robot prim and that Play has been pressed.

## API services and ports

The extension starts one task API and one API per discovered robot:

- Task API: base port `8001`
- First robot: `8002`
- Second robot: `8003`
- Additional robots continue incrementing ports

Robot discovery follows the [supported robot prims](#supported-robot-prims) naming rules. Swagger UI is available for each active service at `http://127.0.0.1:<port>/docs`.

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

## Quick API examples

Ping the task API:

```bash
curl http://127.0.0.1:8001/ping
```

Ping the first robot:

```bash
curl http://127.0.0.1:8002/ping
```

Get robot status:

```bash
curl http://127.0.0.1:8002/status
```

Move a ground robot forward:

```bash
curl -X POST "http://127.0.0.1:8002/move?meters=1.0"
```

Rotate a ground robot:

```bash
curl -X POST "http://127.0.0.1:8002/rotate?deg=90"
```

Stop the robot:

```bash
curl -X POST http://127.0.0.1:8002/stop
```

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

## Common problems

### `http://127.0.0.1:8002/docs` does not open

Check:

- Isaac Sim is still running.
- The extension is enabled.
- Play has been pressed.
- The stage contains a supported robot prim directly under the stage root.
- The robot prim is named `Spot`, `Spot-1`, `Drone`, `Drone-1`, etc.

### Robot control works, but `/frame` or `/sensors` does not

Motion control can work without sensors. For sensor endpoints, the robot should contain:

- `{robot_root}/body/FrontCam`
- `{robot_root}/body/Sensors`

If those prims are missing or named differently, the extension will skip those sensors.

### `/target` does not return useful data

The task target is expected at:

- `{stage_root}/Environment/Target`

If the stage does not include this prim, robot APIs may still work, but task-target functionality will be unavailable.
