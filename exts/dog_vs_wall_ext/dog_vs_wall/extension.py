import queue
import asyncio
import math
import numpy as np

import omni.usd
import omni.ext
import omni.kit.app
import omni.timeline
import omni.physx as physx
from pxr import UsdGeom, Gf
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

from sim_core.utils import log
from .drone_control import DroneRuntime
from sim_core.spot_control import SpotRuntime
from sim_core.task_control import TaskRuntime
from sim_core.api_server import start_spot_api, start_drone_api, start_task_api

WORLD_PATH = "/World"
HOST = "127.0.0.1"

# Spot
SPOT_BASE_POSITION = np.array([0.0, 0.0, 0.8], dtype=np.float32)
SPOT_BASE_ROTATION_DEG = Gf.Vec3f(0.0, 0.0, 0.0)
SPOT_PATH = WORLD_PATH + "/Spot"
SPOT_BODY_PATH = SPOT_PATH + "/body"
SPOT_CAM_PATH = SPOT_BODY_PATH + "/Spot_Cam"
SPOT_IMU_PATH = SPOT_BODY_PATH + "/Imu_Sensor"
SPOT_PORT = 8001

# Drone
DRONE_BASE_POSITION = Gf.Vec3d(0.5, -6.0, 3.5)
DRONE_BASE_ROTATION_DEG = Gf.Vec3f(0.0, 0.0, 90.0)
DRONE_PATH = WORLD_PATH + "/Drone"
DRONE_BODY_PATH = DRONE_PATH + "/body"
DRONE_CAM_PATH = DRONE_BODY_PATH + "/Drone_Cam"
DRONE_IMU_PATH = None
DRONE_PORT = 8002

# Camera
CAM_RES = (640, 480)
SENSOR_HZ = 5.0

# Environment
ENVIRONMENT_PATH = WORLD_PATH + "/Environment"
TARGET_PATH = ENVIRONMENT_PATH + "/Target"
TASK_PORT = 8003



class Extension(omni.ext.IExt):

    def on_startup(self, ext_id: str):
        log("STARTUP", 2)

        # Runtime state
        self._inited = False
        self._physx_sub = None
        self._physx_iface = None
        self.spot = None
        self._task_reset_needed = False

        # Commands + runtime
        self.spot_cmd_q = queue.Queue()
        self.spot_runtime = SpotRuntime(
            cmd_q=self.spot_cmd_q,
            spot_body_path=SPOT_BODY_PATH,
            cam_path=SPOT_CAM_PATH,
            imu_path=SPOT_IMU_PATH,
            cam_res=CAM_RES,
            sensor_hz=SENSOR_HZ,
        )
        self.drone_cmd_q = queue.Queue()
        self.drone_runtime = DroneRuntime(
            cmd_q=self.drone_cmd_q,
            drone_path=DRONE_PATH,
            drone_body_path=DRONE_BODY_PATH,
            cam_path=DRONE_CAM_PATH,
            imu_path=DRONE_IMU_PATH,
            cam_res=CAM_RES,
            sensor_hz=SENSOR_HZ,
        )
        self.task_runtime = TaskRuntime(target_path=TARGET_PATH)

        # API servers
        self._spot_server, self._spot_api_thread = start_spot_api(
            self.spot_cmd_q, HOST, SPOT_PORT,
            get_status=self.spot_runtime.get_status,
            get_pose=self.spot_runtime.get_pose,
            get_sensors=self.spot_runtime.get_sensors,
            get_frame=self.spot_runtime.get_frame
        ) 
        log(f"Spot api on http://{HOST}:{SPOT_PORT}", 2)

        self._drone_server, self._drone_api_thread = start_drone_api(
            self.drone_cmd_q, HOST, DRONE_PORT,
            get_status=self.drone_runtime.get_status,
            get_sensors=self.drone_runtime.get_sensors,
            get_frame=self.drone_runtime.get_frame
        ) 
        log(f"Drone api on http://{HOST}:{DRONE_PORT}", 2)

        self._task_server, self._task_api_thread = start_task_api(
            HOST, TASK_PORT,
            get_target=self.task_runtime.get_target,
            do_reset=self._request_task_reset
        )
        log(f"Task api on http://{HOST}:{TASK_PORT}", 2)

        # Timeline subscription
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(self._on_timeline_event)

    def on_shutdown(self):
        log("SHUTDOWN", 2)

        # Stop API
        try:
            if getattr(self, "_spot_server", None):
                self._spot_server.should_exit = True
            if getattr(self, "_drone_server", None):
                self._drone_server.should_exit = True
            if getattr(self, "_task_server", None):
                self._task_server.should_exit = True
        except Exception:
            pass

        # Unsubscribe PhysX
        try:
            self._physx_sub = None
        except Exception:
            pass

        # Clean self variables
        self._timeline_sub = None
        self._spot_api_thread = None
        self._drone_api_thread = None
        self._task_api_thread = None
        self.spot_runtime = None
        self.drone_runtime = None
        self.task_runtime = None

        self.spot = None
        self._inited = False

    def _on_timeline_event(self, event):
        # Stop -> request reset for next Play
        if not self._timeline.is_playing():
            if self.spot_runtime is not None:
                self.spot_runtime.request_reset()
            if self.drone_runtime is not None:
                self.drone_runtime.request_reset()
            if self.task_runtime is not None:
                self.task_runtime.request_reset()

        # First-ever Play -> spawn + subscribe
        if (not self._inited) and self._timeline.is_playing():
            self._inited = True
            asyncio.ensure_future(self._init_after_play())

    def _request_task_reset(self):
        """ API-thread safe hook: schedule reset on next physics tick """
        self._task_reset_needed = True
        if self.task_runtime is not None:
            self.task_runtime.reset()

    def _clear_cmd_queue(self, cmd_q: "queue.Queue"):
        while True:
            try:
                cmd_q.get_nowait()
            except queue.Empty:
                break

    def _set_prim_pose(
        self,
        prim_path: str,
        position: Gf.Vec3d,
        rotation_deg: Gf.Vec3f,
        label: str,
        required: bool = True
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
        spot_ok = True
        if self.spot_runtime is not None:
            spot_ok = self.spot_runtime.teleport_base(
                x=float(SPOT_BASE_POSITION[0]),
                y=float(SPOT_BASE_POSITION[1]),
                z=float(SPOT_BASE_POSITION[2]),
                yaw_rad=math.radians(float(SPOT_BASE_ROTATION_DEG[2])),
            )
        drone_ok = self._set_prim_pose(
            prim_path=DRONE_PATH,
            position=DRONE_BASE_POSITION,
            rotation_deg=DRONE_BASE_ROTATION_DEG,
            label="DRONE",
            required=False
        )
        return spot_ok and drone_ok

    def _reset_experiment(self):
        self._clear_cmd_queue(self.spot_cmd_q)
        self._clear_cmd_queue(self.drone_cmd_q)
        poses_ok = self._restore_base_poses()

        if self.spot_runtime is not None:
            self.spot_runtime.request_reset()
        if self.drone_runtime is not None:
            self.drone_runtime.request_reset()
        if self.task_runtime is not None:
            self.task_runtime.request_reset()

        if poses_ok:
            log("[TASK] reset experiment: restored base poses", 2)
        else:
            log("[TASK] reset experiment: pose restore partially failed", 3)

    async def _init_after_play(self):
        # Wait for PhysX
        for _ in range(120):  # ~2 seconds
            if physx.get_physx_interface() is not None:
                break
            await omni.kit.app.get_app().next_update_async()
        else:
            log("PhysX not ready", 3)
            return

        # Spawn Spot
        self.spot = SpotFlatTerrainPolicy(
            prim_path=SPOT_PATH,
            name="Spot",
            position=SPOT_BASE_POSITION
        )

        # Spawn drone + normalize both robots to extension-owned base poses
        self._restore_base_poses()

        # Attach sensors/runtime
        self.spot_runtime.attach_spot(self.spot)
        self.drone_runtime.attach_drone()

        # Subscribe physics step events
        self._physx_iface = physx.get_physx_interface()
        try:
            self._physx_sub = self._physx_iface.subscribe_physics_step_events(self._on_world_physics_step)
        except Exception as e:
            log(f"Physx subscribe failed: {e}", 3)

    def _on_world_physics_step(self, step_size: float):
        if not self._timeline.is_playing():
            return
        if self.spot_runtime is None or self.drone_runtime is None or self.task_runtime is None:
            return

        if self._task_reset_needed:
            self._task_reset_needed = False
            self._reset_experiment()

        self.spot_runtime.step(float(step_size))
        self.drone_runtime.step(float(step_size))
        self.task_runtime.step(float(step_size))