# pyright: reportMissingImports=false
import asyncio
import math
import queue

from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
import omni.ext
import omni.kit.app
import omni.physx as physx
import omni.timeline
import omni.usd
from pxr import UsdGeom, Gf

from .api_server import start_spot_api, start_drone_api, start_task_api, stop_api_server
from .constants import (
    BASE_PORT,
    BODY_PRIM,
    DRONE_BASE_POSITION,
    DRONE_BASE_ROTATION_DEG,
    ENVIRONMENT_PRIM,
    FRONT_CAM_PRIM,
    HOST,
    SENSORS_PRIM,
    SPOT_BASE_POSITION,
    SPOT_BASE_ROTATION_DEG,
    TARGET_PRIM,
)
from .drone_control import DroneRuntime
from .spot_control import SpotRuntime
from .task_control import TaskRuntime
from .utils import log, get_stage_root, discover_robots



class Extension(omni.ext.IExt):

    def on_startup(self, ext_id: str):
        log("============================== STARTUP ==============================", 2)

        # Stage state
        self.stage = None
        self.stage_root = None
        self.target_path = None

        # Services
        self.task_runtime = None
        self.robots = []
        self._api_servers = []
        self._discovered = []

        # Runtime state
        self._physx_sub = None
        self._physx_iface = None
        self._task_reset_needed = False
        self._bound_stage_root = None
        self._play_init_running = False

        # Log Flags
        self._last_logged_stage_root = None
        self._play_logged = False

        # TImeline Subscription
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(self._on_timeline_event)

        usd_context = omni.usd.get_context()
        self._stage_sub = usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event,
            name="robot_controller_stage",
        )

        if self._refresh_stage():
            self._sync_services_if_needed(discover_robots(self.stage, self.stage_root))
        else:
            log("No stage open yet; waiting for stage open event", 2)

    def on_shutdown(self):
        log("============================== SHUTDOWN ==============================", 2)
        self._teardown_services()

        try:
            self._physx_sub = None
        except Exception:
            pass

        self._timeline_sub = None
        self._stage_sub = None
        self.stage = None
        self.stage_root = None
        self._bound_stage_root = None
        self._play_logged = False

    # ----- Services -----
    def _setup_services(self, discovered_robots):
        """ Create task API first, then one runtime + API per discovered robot """
        if not self.stage_root:
            log("Cannot setup services: stage root not resolved", 3)
            return

        self._discovered = list(discovered_robots)

        self.target_path = f"{self.stage_root}/{ENVIRONMENT_PRIM}/{TARGET_PRIM}"

        self.task_runtime = TaskRuntime(target_path=self.target_path)
        task_port = BASE_PORT
        task_server, task_thread = start_task_api(
            HOST,
            task_port,
            get_target=self.task_runtime.get_target,
            do_reset=self._request_task_reset,
        )
        self._api_servers.append({
            "kind": "task",
            "name": "task",
            "port": task_port,
            "server": task_server,
            "thread": task_thread,
        })
        log(f"Task api on http://{HOST}:{task_port}", 2)

        self.robots = []
        for i, spec in enumerate(discovered_robots):
            port = BASE_PORT + 1 + i
            robot = self._create_robot_service(spec, port)
            self.robots.append(robot)
            log(
                f"{spec['name']} api on http://{HOST}:{port} -> {spec['path']}",
                2,
            )

        if not discovered_robots:
            log("no robots discovered under stage root", 2)

    def _create_robot_service(self, spec, port):
        kind = spec["kind"]
        path = spec["path"]
        cmd_q = queue.Queue()
        body_path = f"{path}/{BODY_PRIM}"
        cam_path = f"{body_path}/{FRONT_CAM_PRIM}"
        sensors_path = f"{body_path}/{SENSORS_PRIM}"

        if kind == "spot":
            runtime = SpotRuntime(
                cmd_q=cmd_q,
                spot_body_path=body_path,
                cam_path=cam_path,
                imu_path=sensors_path,
            )
            server, thread = start_spot_api(
                cmd_q,
                HOST,
                port,
                get_status=runtime.get_status,
                get_pose=runtime.get_pose,
                get_sensors=runtime.get_sensors,
                get_frame=runtime.get_frame,
            )
        elif kind == "drone":
            runtime = DroneRuntime(
                cmd_q=cmd_q,
                drone_path=path,
                drone_body_path=body_path,
                cam_path=cam_path,
                imu_path=sensors_path,
            )
            server, thread = start_drone_api(
                cmd_q,
                HOST,
                port,
                get_status=runtime.get_status,
                get_sensors=runtime.get_sensors,
                get_frame=runtime.get_frame,
            )
        else:
            raise ValueError(f"unknown robot kind: {kind}")

        self._api_servers.append({
            "kind": kind,
            "name": spec["name"],
            "port": port,
            "server": server,
            "thread": thread,
        })

        return {
            "kind": kind,
            "name": spec["name"],
            "path": path,
            "port": port,
            "cmd_q": cmd_q,
            "runtime": runtime,
            "policy": None,
        }

    def _teardown_services(self):
        for entry in self._api_servers:
            try:
                stop_api_server(entry.get("server"))
            except Exception:
                pass

        self._api_servers = []
        self.robots = []
        self.task_runtime = None
        self._discovered = []
        self._bound_stage_root = None

    def _sync_services_if_needed(self, discovered) -> bool:
        """ Rebuild HTTP services only when stage root or robot discovery changed """
        if not self.stage_root:
            return False

        if (
            self._api_servers
            and self._bound_stage_root == self.stage_root
            and not self._discovery_changed(discovered)
        ):
            return False

        if self._api_servers:
            self._teardown_services()
        self._setup_services(discovered)
        self._bound_stage_root = self.stage_root
        return True

    def _robots_attached(self) -> bool:
        if not self.robots:
            return True
        for robot in self.robots:
            if robot["kind"] == "spot" and robot.get("policy") is None:
                return False
            if robot["kind"] == "drone" and not robot["runtime"].is_attached():
                return False
        return True

    def _discovery_changed(self, discovered):
        if len(discovered) != len(self._discovered):
            return True
        for old, new in zip(self._discovered, discovered):
            if old["path"] != new["path"] or old["kind"] != new["kind"]:
                return True
        return False

    # ----- Stage -----
    def _refresh_stage(self) -> bool:
        """Cache the active USD stage and resolved stage root."""
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            log("No USD stage open", 3)
            self.stage_root = None
            self._last_logged_stage_root = None
            return False

        self.stage_root = get_stage_root(self.stage)
        if not self.stage_root:
            log(
                "stage root could not be resolved "
                "(set defaultPrim or add a PhysicsScene under the stage root)",
                3,
            )
            return False

        if self._last_logged_stage_root != self.stage_root:
            log(f"Stage root: {self.stage_root}", 2)
            self._last_logged_stage_root = self.stage_root
        return True

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.CLOSED):
            self._on_stage_closed()
        elif event.type == int(omni.usd.StageEventType.OPENED):
            self._on_stage_opened()

    def _on_stage_closed(self):
        log("stage closed", 2)
        self._teardown_services()
        self.stage = None
        self.stage_root = None
        self._last_logged_stage_root = None
        self._play_logged = False
        self.target_path = None
        self._physx_sub = None

    def _on_stage_opened(self):
        log("stage opened", 2)
        if not self._refresh_stage():
            log("stage opened but stage root not resolved", 3)
            return

        discovered = discover_robots(self.stage, self.stage_root)
        rebuilt = self._sync_services_if_needed(discovered)
        if (rebuilt or not self._robots_attached()) and self._timeline.is_playing():
            asyncio.ensure_future(self._init_after_play())

    # ----- Timeline -----
    def _on_timeline_event(self, event):
        # Stop -> request reset for next Play
        if not self._timeline.is_playing():
            self._play_logged = False
            for robot in self.robots:
                robot["runtime"].request_reset()
            if self.task_runtime is not None:
                self.task_runtime.request_reset()
            return

        if not self._play_logged:
            log("============================== PLAY ==============================", 2)
            self._play_logged = True
        asyncio.ensure_future(self._init_after_play())

    def _request_task_reset(self):
        """ API-thread safe hook: schedule reset on next physics tick """
        self._task_reset_needed = True
        if self.task_runtime is not None:
            self.task_runtime.reset()

    # ----- Commands -----
    def _clear_cmd_queue(self, cmd_q: "queue.Queue"):
        while True:
            try:
                cmd_q.get_nowait()
            except queue.Empty:
                break

    # ----- Robot Control -----
    def _set_prim_pose(
        self,
        prim_path: str,
        position: Gf.Vec3d,
        rotation_deg: Gf.Vec3f,
        label: str,
        required: bool = True,
    ) -> bool:
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            if required:
                log(f"{label} prim missing at {prim_path}", 3)
                return False
            log(f"{label} prim missing at {prim_path} (optional)", 2)
            return True

        try:
            xform_api = UsdGeom.XformCommonAPI(prim)
            xform_api.SetTranslate(position)
            xform_api.SetRotate(rotation_deg)
            return True
        except Exception as e:
            log(f"{label} pose restore failed at {prim_path}: {e}", 3)
            return False

    def _restore_base_poses(self):
        ok = True
        for robot in self.robots:
            if robot["kind"] == "spot":
                spot_ok = robot["runtime"].teleport_base(
                    x=float(SPOT_BASE_POSITION[0]),
                    y=float(SPOT_BASE_POSITION[1]),
                    z=float(SPOT_BASE_POSITION[2]),
                    yaw_rad=math.radians(float(SPOT_BASE_ROTATION_DEG[2])),
                )
                ok = ok and spot_ok
            elif robot["kind"] == "drone":
                drone_ok = self._set_prim_pose(
                    prim_path=robot["path"],
                    position=DRONE_BASE_POSITION,
                    rotation_deg=DRONE_BASE_ROTATION_DEG,
                    label=robot["name"].upper(),
                    required=False,
                )
                ok = ok and drone_ok
        return ok

    def _reset_experiment(self):
        for robot in self.robots:
            self._clear_cmd_queue(robot["cmd_q"])

        poses_ok = self._restore_base_poses()

        for robot in self.robots:
            robot["runtime"].request_reset()
        if self.task_runtime is not None:
            self.task_runtime.request_reset()

        if poses_ok:
            log("[TASK] reset experiment: restored base poses", 2)
        else:
            log("[TASK] reset experiment: pose restore partially failed", 3)

    # ----- Play Init -----
    async def _init_after_play(self):
        if self._play_init_running:
            return
        self._play_init_running = True
        try:
            await self._run_play_init()
        finally:
            self._play_init_running = False

    async def _run_play_init(self):
        if not self._refresh_stage():
            log("play init aborted: stage root required", 3)
            return

        discovered = discover_robots(self.stage, self.stage_root)
        services_rebuilt = self._sync_services_if_needed(discovered)

        if not self.robots:
            return

        if not services_rebuilt and self._robots_attached():
            return

        # Wait for PhysX
        for _ in range(120):  # ~2 seconds
            if physx.get_physx_interface() is not None:
                break
            await omni.kit.app.get_app().next_update_async()
        else:
            log("PhysX not ready", 3)
            return

        if self._physx_sub is None:
            self._physx_iface = physx.get_physx_interface()
            try:
                self._physx_sub = self._physx_iface.subscribe_physics_step_events(
                    self._on_world_physics_step
                )
            except Exception as e:
                log(f"Physx subscribe failed: {e}", 3)
                return

        for robot in self.robots:
            try:
                if robot["kind"] == "spot" and robot.get("policy") is None:
                    robot["policy"] = SpotFlatTerrainPolicy(
                        prim_path=robot["path"],
                        name=robot["name"],
                        position=SPOT_BASE_POSITION,
                    )
                    robot["runtime"].attach_spot(robot["policy"])
                    log(f"{robot['name']} policy attached", 2)
                elif robot["kind"] == "drone" and not robot["runtime"].is_attached():
                    robot["runtime"].attach_drone()
                    log(f"{robot['name']} attached", 2)
            except Exception as e:
                log(f"{robot['name']} play init failed: {e}", 3)

        self._restore_base_poses()

    def _on_world_physics_step(self, step_size: float):
        if not self._timeline.is_playing():
            return

        if self._task_reset_needed:
            self._task_reset_needed = False
            self._reset_experiment()

        for robot in self.robots:
            robot["runtime"].step(float(step_size))

        if self.task_runtime is not None:
            self.task_runtime.step(float(step_size))
