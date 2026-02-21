import queue
import carb
import numpy as np

import omni.ext
import omni.physx
import omni.timeline
import omni.kit.app

from omni.isaac.core import World
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

from .api_server import start_api
host = "127.0.0.1"
port = 8001

SPOT_PATH = "/World/Spot"

def log(msg):
    carb.log_warn(f"[spot-ext] {msg}")

class Extension(omni.ext.IExt):

    def on_startup(self, ext_id: str):
        log("startup")
        self._inited = False
        self._first_step = True
        self._reset_needed = False
        self._move_active = False

        self._policy_ready = False
        self._policy_inited = False
        self._base_cmd = np.zeros(3)    # vx, vy, vz
        self._move_remaining = 0.0      # meters lef to move
        self._move_speed = 0.5          # m/s
        
        # Start API server
        self.cmd_q = queue.Queue()
        self._server, self._api_thread = start_api(self.cmd_q, host, port)
        log(f"api on http://{host}:{port}")

        # Suscribe to timeline
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(self._on_timeline_event)

    def on_shutdown(self):
        log("shutdown")

        if self._server:
            self._server.should_exit = True
            log("api shutdown sent")

        # remove callback
        try:
            if getattr(self, "world", None) and hasattr(self, "_cb_name"):
                self.world.remove_physics_callback(self._cb_name)
        except Exception as e:
            log(f"remove_physics_callback failed: {e}")

        # clear refs
        self._timeline_sub = None
        self._api_thread = None
        self.spot = None
        self.world = None
        self._inited = False

    def _on_timeline_event(self, event):
        if (not self._inited) and self._timeline.is_playing():
            self._inited = True
            import asyncio
            asyncio.ensure_future(self._init_after_play())

    async def _init_after_play(self):
        await omni.kit.app.get_app().next_update_async()

        self.world = World.instance()
        self.world.reset()

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        
        # Ground
        try: 
            self.world.scene.add_default_ground_plane()
        except Exception as e:
            log(f"Ground plane already exists? {e}")

        # Spawn spot
        self.spot = SpotFlatTerrainPolicy(
            prim_path=SPOT_PATH,
            name="Spot",
            position=np.array([0.0, 0.0, 0.8])
        )
        log("Spot spawned")

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        
        # Start command loop
        self._cb_name = "spot_policy_step"
        try:
            self.world.remove_physics_callback(self._cb_name)
        except Exception:
            pass
        self.world.add_physics_callback(self._cb_name, self._on_world_physics_step)
        log("Physiscs step suscribed")


    # ----- Main loop -----
    def _on_world_physics_step(self, step_size: float):

        if not hasattr(self, "_hb"):
            self._hb = 0
        self._hb += 1
        if self._hb % 120 == 0:  # ~every 2 seconds at 60Hz
            log(f"heartbeat step_size={step_size}")

        if self._first_step:
            self.spot.initialize()
            self._first_step = False
            log("Spot policy initialized")
            return

        # Process commands in queue
        while True:
            try:
                cmd = self.cmd_q.get_nowait()
                log(f"got cmd: {cmd}")
            except queue.Empty:
                break

            if cmd[0] == "cmd_vel":
                _, vx, vy, wz = cmd
                self._base_cmd[:] = [vx, vy, wz]
                log(f"Applied cmd_vel: vx={vx}, vy={vy}, wz={wz}]")
            elif cmd[0] == "move":
                meters = float(cmd[1])
                self._move_remaining += meters
                self._move_active = True
                log(f"Queued move: {meters}m (remaining={self._move_remaining}m)")

        dt = float(step_size)

        # Move goal
        if self._move_active:
            if abs(self._move_remaining) > 1e-4:
                vx = self._move_speed * (1.0 if self._move_remaining > 0 else -1.0)
                self._base_cmd[:] = [vx, 0.0, 0.0]
                self._move_remaining -= vx * dt
            else:
                self._base_cmd[:] = [0.0, 0.0, 0.0]
                self._move_active = False

        # Locomotion
        try: 
            self.spot.forward(dt, self._base_cmd)
        except Exception as e:
            log(f"spot.forward failed: {e}")