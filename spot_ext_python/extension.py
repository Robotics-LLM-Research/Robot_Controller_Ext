import queue
import asyncio
import numpy as np

import omni.usd
import omni.ext
import omni.kit.app
import omni.timeline
import omni.physx as physx
from pxr import UsdGeom, Gf
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

from .utils import log
from .spot_control import SpotRuntime
from .drone_control import DroneRuntime
from .api_server import start_spot_api, start_drone_api

WORLD_PATH = "/World"
HOST = "127.0.0.1"

# Spot
SPOT_PATH = WORLD_PATH + "/Spot"
SPOT_BODY_PATH = SPOT_PATH + "/body"
SPOT_CAM_PATH = SPOT_BODY_PATH + "/Spot_Cam"
SPOT_IMU_PATH = SPOT_BODY_PATH + "/Imu_Sensor"
SPOT_PORT = 8001

# Drone
DRONE_PATH = WORLD_PATH + "/Crazyflie"
DRONE_BODY_PATH = DRONE_PATH + "/body"
DRONE_CAM_PATH = DRONE_BODY_PATH + "/Crazyflie_Cam"
DRONE_IMU_PATH = None
DRONE_PORT = 8002

# Camera
CAM_RES = (640, 480)
SENSOR_HZ = 5.0



class Extension(omni.ext.IExt):

    def on_startup(self, ext_id: str):
        log("STARTUP", 2)

        # Runtime state
        self._inited = False
        self._physx_sub = None
        self._physx_iface = None
        self.spot = None

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

        # API servers
        self._spot_server, self._spot_api_thread = start_spot_api(
            self.spot_cmd_q, HOST, SPOT_PORT,
            get_status=self.spot_runtime.get_status,
            get_sensors=self.spot_runtime.get_sensors
        ) 
        log(f"Spot api on http://{HOST}:{SPOT_PORT}", 2)

        self._drone_server, self._drone_api_thread = start_drone_api(
            self.drone_cmd_q, HOST, DRONE_PORT,
            get_status=self.drone_runtime.get_status,
            get_sensors=self.drone_runtime.get_sensors
        ) 
        log(f"Drone api on http://{HOST}:{DRONE_PORT}", 2)

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
        self.spot_runtime = None
        self.drone_runtime = None

        self.spot = None
        self._inited = False

    def _on_timeline_event(self, event):
        # Stop -> request reset for next Play
        if not self._timeline.is_playing():
            if self.spot_runtime is not None:
                self.spot_runtime.request_reset()
            if self.drone_runtime is not None:
                self.drone_runtime.request_reset()

        # First-ever Play -> spawn + subscribe
        if (not self._inited) and self._timeline.is_playing():
            self._inited = True
            asyncio.ensure_future(self._init_after_play())

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
            position=np.array([0.0, 0.0, 0.8])
        )

        # Spawn drone
        stage = omni.usd.get_context().get_stage()
        drone_prim = stage.GetPrimAtPath(DRONE_PATH)
        if drone_prim and drone_prim.IsValid():
            UsdGeom.XformCommonAPI(drone_prim).SetTranslate(Gf.Vec3d(0.5, -6.0, 3.5))
            UsdGeom.XformCommonAPI(drone_prim).SetRotate(Gf.Vec3f(0.0, 0.0, 90.0))  # level
        else:
            log(f"DRONE BODY prim missing at {DRONE_BODY_PATH}", 3)

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
        if self.spot_runtime is None or self.drone_runtime is None:
            return
        self.spot_runtime.step(float(step_size))
        self.drone_runtime.step(float(step_size))