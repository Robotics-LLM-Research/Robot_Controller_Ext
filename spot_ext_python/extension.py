import asyncio
import queue
import numpy as np

import carb
import omni.ext
import omni.kit.app
import omni.timeline
import omni.physx as physx

from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

from .api_server import start_api
from .control import SpotRuntime
from .utils import log

HOST = "127.0.0.1"
PORT = 8001

SPOT_PATH = "/World/Spot"
SPOT_BODY_PATH = "/World/Spot/body"

LIDAR_ORIGIN = SPOT_PATH + "/body"
CAM_PATH = SPOT_PATH + "/body/Spot_Front_Cam"
IMU_PATH = SPOT_PATH + "/body/Imu_Sensor"

CAM_RES = (640, 480)
SENSOR_HZ = 5.0



class Extension(omni.ext.IExt):

    def on_startup(self, ext_id: str):
        log("startup")

        # Runtime state
        self._inited = False
        self._physx_sub = None
        self._physx_iface = None
        self.spot = None

        # Commands + runtime
        self.cmd_q = queue.Queue()
        self.runtime = SpotRuntime(
            cmd_q=self.cmd_q,
            spot_body_path=SPOT_BODY_PATH,
            lidar_origin_path=LIDAR_ORIGIN,
            cam_path=CAM_PATH,
            imu_path=IMU_PATH,
            cam_res=CAM_RES,
            sensor_hz=SENSOR_HZ,
        )

        # API server (unchanged endpoints)
        self._server, self._api_thread = start_api(
            self.cmd_q, HOST, PORT,
            get_lidar=self.runtime.get_lidar,
            get_sensors=self.runtime.get_sensors
        )
        log(f"api on http://{HOST}:{PORT}")

        # Timeline subscription
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(self._on_timeline_event)

    def on_shutdown(self):
        log("shutdown")

        # Stop API
        try:
            if getattr(self, "_server", None):
                self._server.should_exit = True
        except Exception:
            pass

        # Unsubscribe PhysX
        try:
            self._physx_sub = None
        except Exception:
            pass

        self._timeline_sub = None
        self._api_thread = None
        self.runtime = None
        self.spot = None
        self._inited = False

    def _on_timeline_event(self, event):
        # Stop -> request reset for next Play
        if not self._timeline.is_playing():
            if self.runtime is not None:
                self.runtime.request_reset()

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
            log("PhysX not ready")
            return

        # Spawn Spot policy robot
        self.spot = SpotFlatTerrainPolicy(
            prim_path=SPOT_PATH,
            name="Spot",
            position=np.array([0.0, 0.0, 0.8])
        )
        log("Spot spawned")

        # Attach spot + init sensors (camera/imu) inside runtime
        self.runtime.attach_spot(self.spot)

        # Subscribe physics step events
        self._physx_iface = physx.get_physx_interface()
        try:
            self._physx_sub = self._physx_iface.subscribe_physics_step_events(self._on_world_physics_step)
            log("physx step subscribed")
        except Exception as e:
            log(f"physx subscribe failed: {e}")

    def _on_world_physics_step(self, step_size: float):
        if not self._timeline.is_playing():
            return
        if self.runtime is None:
            return
        self.runtime.step(float(step_size))