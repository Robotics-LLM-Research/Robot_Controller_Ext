import queue
import carb
import omni.ext
import omni.physx
import omni.timeline

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

from .api_server import start_api
host = "127.0.0.1"
port = 8001

SPOT_PATH = "/World/spot_with_arm"

def log(msg):
    carb.log_warn(f"[spot-ext] {msg}")

class Extension(omni.ext.IExt):

    def on_startup(self, ext_id: str):
        log("startup")
        self._inited = False
        
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

        self._timeline_sub = None
        self._api_thread = None
        self.spot = None
        self.world = None
        self._physx_sub = None
        
        self._inited = False

    def _on_timeline_event(self, event):
        if (not self._inited) and self._timeline.is_playing():
            import asyncio
            asyncio.ensure_future(self._init_after_play())

    async def _init_after_play(self):
        # Wait 1 frame to ensure everything is loaded and ready
        await omni.kit.app.get_app().next_update_async()

        self.world = World()
        self.world.reset()

        self.spot = Articulation(SPOT_PATH)
        self.spot.initialize()

        log(f"DOF={self.spot.num_dof}")
        self._inited = True

        # Start command loop
        self._physx = omni.physx.get_physx_interface()
        self._physx_sub = self._physx.subscribe_physics_step_events(self._on_physics_step)
        log("Physiscs step suscribed")

    def _on_physics_step(self, step):
        if not hasattr(self, "_tick_logged"):
            log("physics ticks running")
            self._tick_logged = True

        if not self._inited:
            return
        
        # Process commands in queue
        while True:
            try:
                cmd = self.cmd_q.get_nowait()
            except queue.Empty:
                break

            if cmd[0] == "joint_delta":
                _, joint, delta = cmd
                q = self.spot.get_joint_positions()
                q[joint] += delta
                self.spot.apply_action(ArticulationAction(joint_positions=q))
                log(f"Applied joint delta: joint={joint}, delta={delta}")

    async def _init_robot(self):
        if getattr(self, "_inited", False):
            return
        await omni.kit.app.get_app().next_update_async()
        self.world.reset()
        self.spot.initialize()