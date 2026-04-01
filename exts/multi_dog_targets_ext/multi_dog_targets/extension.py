import asyncio
import math
import queue

import numpy as np
import omni.ext
import omni.kit.app
import omni.physx as physx
import omni.timeline
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from sim_core.api_server import start_spot_api, start_task_api
from sim_core.spot_control import SpotRuntime
from sim_core.task_control import TaskRuntime
from sim_core.utils import log
from pxr import Gf

WORLD_PATH = "/World"
HOST = "127.0.0.1"

# Environment
ENVIRONMENT_PATH = WORLD_PATH + "/Environment"
TARGET_PATH = ENVIRONMENT_PATH + "/Target"
TASK_PORT = 8000

# Dogs
DOGS = [
    {
        "name": "dog1",
        "prim_path": WORLD_PATH + "/dog1",
        "body_path": WORLD_PATH + "/dog1/body",
        "cam_path": WORLD_PATH + "/dog1/body/dog1_cam",
        "imu_path": WORLD_PATH + "/dog1/body/Imu_Sensor",
        "port": 8001,
        "pos": np.array([0.0, 0.0, 0.8], dtype=np.float32),
        "rot": Gf.Vec3f(0.0, 0.0, 0.0),
    },
    {
        "name": "dog2",
        "prim_path": WORLD_PATH + "/dog2",
        "body_path": WORLD_PATH + "/dog2/body",
        "cam_path": WORLD_PATH + "/dog2/body/dog2_cam",
        "imu_path": WORLD_PATH + "/dog2/body/Imu_Sensor",
        "port": 8002,
        "pos": np.array([0.0, 3.0, 0.8], dtype=np.float32),
        "rot": Gf.Vec3f(0.0, 0.0, 0.0),
    },
    {
        "name": "dog3",
        "prim_path": WORLD_PATH + "/dog3",
        "body_path": WORLD_PATH + "/dog3/body",
        "cam_path": WORLD_PATH + "/dog3/body/dog3_cam",
        "imu_path": WORLD_PATH + "/dog3/body/Imu_Sensor",
        "port": 8003,
        "pos": np.array([0.0, 6.0, 0.8], dtype=np.float32),
        "rot": Gf.Vec3f(0.0, 0.0, 0.0),
    },
    {
        "name": "dog4",
        "prim_path": WORLD_PATH + "/dog4",
        "body_path": WORLD_PATH + "/dog4/body",
        "cam_path": WORLD_PATH + "/dog4/body/dog4_cam",
        "imu_path": WORLD_PATH + "/dog4/body/Imu_Sensor",
        "port": 8004,
        "pos": np.array([0.0, 9.0, 0.8], dtype=np.float32),
        "rot": Gf.Vec3f(0.0, 0.0, 0.0),
    },
    {
        "name": "dog5",
        "prim_path": WORLD_PATH + "/dog5",
        "body_path": WORLD_PATH + "/dog5/body",
        "cam_path": WORLD_PATH + "/dog5/body/dog5_cam",
        "imu_path": WORLD_PATH + "/dog5/body/Imu_Sensor",
        "port": 8005,
        "pos": np.array([0.0, 12.0, 0.8], dtype=np.float32),
        "rot": Gf.Vec3f(0.0, 0.0, 0.0),
    },
]

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
        self._task_reset_needed = False

        self.spots = {}
        self.spot_cmd_qs = {}
        self.spot_runtimes = {}
        self._spot_servers = {}
        self._spot_api_threads = {}

        # Commands + runtime
        for dog in DOGS:
            spot_cmd_q = queue.Queue()
            spot_runtime = SpotRuntime(
                cmd_q=spot_cmd_q,
                spot_body_path=dog["body_path"],
                cam_path=dog["cam_path"],
                imu_path=dog["imu_path"],
                cam_res=CAM_RES,
                sensor_hz=SENSOR_HZ,
            )

            self.spot_cmd_qs[dog["name"]] = spot_cmd_q
            self.spot_runtimes[dog["name"]] = spot_runtime

        self.task_runtime = TaskRuntime(target_path=TARGET_PATH)

        # API servers
        for dog in DOGS:
            spot_runtime = self.spot_runtimes[dog["name"]]
            spot_cmd_q = self.spot_cmd_qs[dog["name"]]

            spot_server, spot_api_thread = start_spot_api(
                spot_cmd_q, HOST, dog["port"],
                get_status=spot_runtime.get_status,
                get_pose=spot_runtime.get_pose,
                get_sensors=spot_runtime.get_sensors,
                get_frame=spot_runtime.get_frame,
            )

            self._spot_servers[dog["name"]] = spot_server
            self._spot_api_threads[dog["name"]] = spot_api_thread
            log(f"{dog['name']} api on http://{HOST}:{dog['port']}", 2)

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
            for spot_server in self._spot_servers.values():
                if spot_server:
                    spot_server.should_exit = True
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
        self._task_api_thread = None
        self._spot_api_threads = {}
        self._spot_servers = {}
        self.spot_runtimes = {}
        self.spot_cmd_qs = {}
        self.spots = {}
        self.task_runtime = None
        self._inited = False

    def _on_timeline_event(self, event):
        # Stop -> request reset for next Play
        if not self._timeline.is_playing():
            for spot_runtime in self.spot_runtimes.values():
                if spot_runtime is not None:
                    spot_runtime.request_reset()
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

    def _restore_base_poses(self):
        spots_ok = True

        for dog in DOGS:
            spot_runtime = self.spot_runtimes.get(dog["name"])
            if spot_runtime is None:
                spots_ok = False
                continue

            spot_ok = spot_runtime.teleport_base(
                x=float(dog["pos"][0]),
                y=float(dog["pos"][1]),
                z=float(dog["pos"][2]),
                yaw_rad=math.radians(float(dog["rot"][2])),
            )
            spots_ok = spots_ok and spot_ok

        return spots_ok

    def _reset_experiment(self):
        for spot_cmd_q in self.spot_cmd_qs.values():
            self._clear_cmd_queue(spot_cmd_q)

        poses_ok = self._restore_base_poses()

        for spot_runtime in self.spot_runtimes.values():
            if spot_runtime is not None:
                spot_runtime.request_reset()

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

        # Spawn Spots
        for dog in DOGS:
            self.spots[dog["name"]] = SpotFlatTerrainPolicy(
                prim_path=dog["prim_path"],
                name=dog["name"],
                position=dog["pos"]
            )

        # Normalize all dogs to extension-owned base poses
        self._restore_base_poses()

        # Attach sensors/runtime
        for dog in DOGS:
            spot_runtime = self.spot_runtimes[dog["name"]]
            spot = self.spots[dog["name"]]
            spot_runtime.attach_spot(spot)

        # Subscribe physics step events
        self._physx_iface = physx.get_physx_interface()
        try:
            self._physx_sub = self._physx_iface.subscribe_physics_step_events(self._on_world_physics_step)
        except Exception as e:
            log(f"Physx subscribe failed: {e}", 3)

    def _on_world_physics_step(self, step_size: float):
        if not self._timeline.is_playing():
            return
        if not self.spot_runtimes or self.task_runtime is None:
            return

        if self._task_reset_needed:
            self._task_reset_needed = False
            self._reset_experiment()

        for spot_runtime in self.spot_runtimes.values():
            spot_runtime.step(float(step_size))

        self.task_runtime.step(float(step_size))