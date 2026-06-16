# Changelog

## [Unreleased]

## [1.0.3] - 2026-06-16
### Added
- Dynamic stage root resolution (`defaultPrim`, PhysicsScene parent, then `/World` or `/Root`).
- Dynamic robot discovery for `Spot`, `Spot-N`, `Drone`, and `Drone-N` prims directly under the stage root.
- Dynamic HTTP port allocation: task API on `8001`, then one API per discovered robot on `8002+` in discovery order.
- `constants.py` for shared stage, robot, sensor, and API configuration.
- Stage open/close event handling so services rebuild when worlds are swapped.
- `stop_api_server()` helper for reliable background API shutdown.
- Optional `FrontCam` and `Sensors` prims — motion control works when sensors are missing.
- Isaac Sim in-app documentation in `docs/README.md` (enable, play flow, stage requirements, endpoint reference).

### Changed
- Refactored into a single `Robot_Controller_ext` extension with Python module `robot_controller`.
- Consolidated shared runtime code into `api_server`, `spot_control`, `drone_control`, `sensing`, `task_control`, and `utils`.
- Standardized prim naming: `body`, `FrontCam`, `Sensors`, `Environment/Target`.
- Standardized terminology from "aerial robot" to "Drone" across code and docs.
- Extension metadata now points the in-app readme to `docs/README.md`.

### Fixed
- Timeline and play-init bugs: duplicate initialization, stale services after stage changes, and robots not re-attaching when reopening a world while playing.
- Logging noise from repeated stage-root messages, missing-camera warnings, and optional-sensor status spam.

### Removed
- Legacy `dog_vs_wall_ext`, `multi_dog_targets_ext`, and multi-dog world assets.
- Unused `agent/` LLM client scripts and `sim_core/` helper module.

## [1.0.2] - 2026-03-04
### Added
- Root README with project overview and Isaac Sim setup walkthrough (clone, launch, activate extension, load world)
- Extension docs README with API description and interactive `/docs` usage for ground robot (8001) and Drone (8002)


## [1.0.1] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings


## [0.1.0] - 2026-02-21
### Added
- Initial version of spot_ext Extension
