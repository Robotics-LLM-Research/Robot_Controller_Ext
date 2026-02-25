import carb
import math
import time
import queue
import numpy as np

import omni.ext
import omni.usd
import omni.kit.app
import omni.timeline
import omni.physx as physx
import omni.replicator.core as rep

from pxr import UsdGeom, Sdf
from isaacsim.core.api import World
from isaacsim.sensors.physics import _sensor
from omni.physx import get_physx_scene_query_interface
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

from .api_server import start_api

HOST = "127.0.0.1"
PORT = 8001

SPOT_PATH = "/World/Spot"
LIDAR_ORIGIN = SPOT_PATH + "/body"
CAM_PATH = SPOT_PATH + "/body/Spot_Front_Cam"
IMU_PATH = SPOT_PATH + "/body/Imu_Sensor"
CAM_RES = (640, 480)
SENSOR_HZ = 5.0

def log(msg):
    carb.log_warn(f"[spot-ext] {msg}")

def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

def _get_world_pose_xy_yaw(prim_path: str):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)

    cache = UsdGeom.XformCache()
    m = cache.GetLocalToWorldTransform(prim)

    # position
    p = m.ExtractTranslation()
    x, y, z = float(p[0]), float(p[1]), float(p[2])

    # yaw from rotation
    r = m.ExtractRotationMatrix()
    yaw = math.atan2(float(r[1][0]), float(r[0][0]))
    return x, y, z, yaw

def raycast_distance_from_yaw(spot_root_path: str, rel_yaw_rad: float, max_dist: float = 10.0):
    x, y, z, yaw = _get_world_pose_xy_yaw(spot_root_path)

    origin = carb.Float3(x, y, z + 0.30)  # lift ray origin
    a = yaw + rel_yaw_rad
    direction = carb.Float3(math.cos(a), math.sin(a), 0.0)

    sq = get_physx_scene_query_interface()
    hit = sq.raycast_closest(origin, direction, max_dist, True)

    if hit and ("distance" in hit):
        return float(hit["distance"]), hit.get("rigid_body", None)
    return max_dist, None

class Extension(omni.ext.IExt):

    def on_startup(self, ext_id: str):
        log("startup")
        self._dbg_i = 0 # DEBUG

        # --- Runtime state ---
        self._inited = False
        self._first_step = True
        self._policy_inited = False
        self._warmup_left = 0
        self._reset_needed = False

        # PhysX
        self._physx_sub = None
        self._physx_iface = None

        # Spot policy
        self.world = None
        self.spot = None

        # Goal based motion
        self._base_cmd = np.zeros(3)

        self._move_active = False
        self._move_remaining = 0.0      # meters lef to move
        self._move_speed = 2            # m/s

        self._rot_active = False
        self._rot_target_yaw = None
        self._rot_kp = 2.0
        self._rot_max_wz = 1.0          # rad/s
        self._rot_tol = math.radians(2) # stop within 2 degrees

        self._hold_heading = True
        self._hold_yaw = None
        self._hold_kp = 1.0
        self._hold_max_wz = 0.2

        # Lidar
        self._lidar_rel_yaw = 0.0       # radians, 0 = forward
        self._lidar_max_dist = 10.0     # meters
        self._lidar_last_dist = self._lidar_max_dist
        self._lidar_last_hit = None

        # Sensors
        self._rp = None
        self._rgb_annot = None
        self._depth_annot = None
        self._last_sensor_t = 0.0
        self._imu_iface = None

        self.sensor_state = {
            "front_clear_m": float("inf"),
            "left_clear_m": float("inf"),
            "right_clear_m": float("inf"),
            "imu_lin_acc": [0.0, 0.0, 0.0],
            "imu_ang_vel": [0.0, 0.0, 0.0],
            "imu_orientation": None,
        }

        def _get_sensors():
            """
            Unit in meters
            Outputs:
                "front_clear_m": closest obstacle directly ahead
                "left_clear_m": closest obstacle in the left region
                "right_clear_m": closest obstacle in the right region
                "imu_lin_acc": IMU linear acceleration [X, Y, Z]
                "imu_ang_vel": IMU rotational velocity [X: roll, Y: pitch, Z: yaw]
                "imu_orientation": Spot's 3D orientation [X, Y, Z, W]
            """
            s = dict(self.sensor_state)
            ori = s.get("imu_orientation", None)
            if ori is not None and not isinstance(ori, (list, tuple)):
                try:
                    s["imu_orientation"] = list(ori)
                except Exception:
                    s["imu_orientation"] = None
            return s

        # --- API Server ---
        self.cmd_q = queue.Queue()
        def _get_lidar():
            return float(self._lidar_last_dist), (str(self._lidar_last_hit) if self._lidar_last_hit else None)
        
        self._server, self._api_thread = start_api(
            self.cmd_q, HOST, PORT, 
            get_lidar=_get_lidar,
            get_sensors=_get_sensors
            )
        log(f"api on http://{HOST}:{PORT}")

        # --- Timeline event ---
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

        # Unsubscribe from physix
        try:
            self._physx_sub = None
        except Exception:
            pass

        # clear refs
        self._timeline_sub = None
        self._api_thread = None
        self.spot = None
        self.world = None
        self._inited = False

    # Timeline event
    def _on_timeline_event(self, event):
        # if timeline is not playing, next Play should re-init policy
        if not self._timeline.is_playing():
            self._reset_needed = True

        # first-ever Play: do init
        if (not self._inited) and self._timeline.is_playing():
            self._inited = True
            import asyncio
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

        # Spawn spot
        self.spot = SpotFlatTerrainPolicy(
            prim_path=SPOT_PATH,
            name="Spot",
            position=np.array([0.0, 0.0, 0.8])
        )
        log("Spot spawned")

        # Init camera
        self._init_camera_depth(CAM_PATH)
        self._init_imu(IMU_PATH)
     
        # Subscribe to physx step events
        self._physx_iface = physx.get_physx_interface()
        try:
            self._physx_sub = self._physx_iface.subscribe_physics_step_events(self._on_world_physics_step)
            log("physx step subscribed")
        except Exception as e:
            log(f"physx subscribe failed: {e}")
            return


    # ----- Main physics step loop -----
    def _on_world_physics_step(self, step_size: float):
        if not self._timeline.is_playing():
            return
        if self.spot is None:
            return
        
        # If we just came back from Stop -> Play, rebuild policy state
        if self._reset_needed:
            self._reset_needed = False

            # clear motion
            self._base_cmd[:] = [0.0, 0.0, 0.0]
            self._move_active = False
            self._rot_active = False
            self._move_remaining = 0.0
            self._rot_remaining_rad = 0.0

            # soft reset
            try:
                self.world.reset(True)
            except Exception as e:
                log(f"world.reset(True) failed on replay: {e}")

            # force policy re-init + warmup
            self._policy_inited = False
            self._warmup_left = 60   # give sim-view time to recreate
            return

        # Initialize policy once
        if not self._policy_inited:
            try:
                self.spot.initialize()
                self._policy_inited = True
                self._warmup_left = 60  # ~10 physics steps of delay
            except Exception as e:
                log(f"spot.initialize not ready yet: {e}")
            return
        
        self._process_cmd_queue()
        
        if self._warmup_left > 0:
            self._warmup_left -= 1
            return

        dt = float(step_size)

        # Update sensors
        self._update_sensors()

        # Drain queue and apply locomotion
        self._process_cmd_queue()
        self._apply_goal_motion(dt)
        try:
            self.spot.forward(dt, self._base_cmd)
        except Exception as e:
            log(f"spot.forward failed: {e}")

        ################################################################################ DEBUG RAYCAST LIDAR
        self._dbg_i += 1
        if self._dbg_i % 30 == 0:
            x, y, z, yaw = _get_world_pose_xy_yaw(SPOT_PATH)
            log(f"ray pose from {SPOT_PATH}: x={x:.2f} y={y:.2f} yaw={yaw:.2f}  lidar={self._lidar_last_dist:.2f}")
            #############################################################################

        ############################################################################# DEBUG ROTATION OFFSET
        if self._dbg_i % 60 == 0:
            x, y, z, yaw = _get_world_pose_xy_yaw("/World/Spot/body")
            log(f"yaw={yaw:.3f}  cmd_wz={self._base_cmd[2]:.4f}")
            #############################################################################

        # Update single-ray lidar
        try:
            d, hit = raycast_distance_from_yaw(LIDAR_ORIGIN, self._lidar_rel_yaw, max_dist=self._lidar_max_dist)
            self._lidar_last_dist = d
            self._lidar_last_hit = hit
        except Exception as e:
            log(f"lidar raycast failed: {e}")     

    # ----- Command processing -----
    def _process_cmd_queue(self):
        while True:
            try:
                cmd = self.cmd_q.get_nowait()
            except queue.Empty:
                break

            kind = cmd[0]

            if kind == "cmd_vel":
                _, vx, vy, wz = cmd
                self._move_active = False
                self._rot_active = False
                self._move_remaining = 0.0
                self._rot_remaining_rad = 0.0

                self._base_cmd[:] = [vx, vy, wz]
                log(f"cmd_vel set: vx={vx}, vy={vy}, wz={wz}")

            elif kind == "move":
                meters = float(cmd[1])
                self._move_remaining += meters
                self._move_active = True
                log(f"move queued: {meters} m (remaining={self._move_remaining} m)")

            elif kind == "rotate":
                deg = float(cmd[1])
                delta = math.radians(deg)
                _, _, _, cur_yaw = _get_world_pose_xy_yaw("/World/Spot/body")

                if self._rot_active and self._rot_target_yaw is not None:
                    self._rot_target_yaw = _wrap_pi(self._rot_target_yaw + delta)
                else:
                    self._rot_target_yaw = _wrap_pi(cur_yaw + delta)
                    self._rot_active = True

                log(f"rotate target set: {deg} deg -> target_yaw={self._rot_target_yaw:.3f}")

            elif kind == "stop":
                self._base_cmd[:] = [0.0, 0.0, 0.0]
                self._move_active = False
                self._rot_active = False
                self._rot_target_yaw = None
                self._move_remaining = 0.0
                self._rot_remaining_rad = 0.0
                log("stop: cleared base cmd + goals")

            elif kind == "lidar_cfg":
                _, yaw_deg, max_dist = cmd
                self._lidar_rel_yaw = math.radians(float(yaw_deg))
                self._lidar_max_dist = float(max_dist)
                log(f"lidar_cfg: yaw_deg={yaw_deg}, max_dist={max_dist}")

            else:
                log(f"unknown cmd: {cmd}")

    def _apply_goal_motion(self, dt: float):
        if (not self._move_active) and (not self._rot_active):
            return
 
        vx = 0.0
        wz = 0.0

        # Move goal
        if self._move_active:
            if abs(self._move_remaining) > 1e-4:
                direction = 1.0 if self._move_remaining > 0 else -1.0
                vx = self._move_speed * direction
                self._move_remaining -= vx * dt
            else:
                self._move_remaining = 0.0
                self._move_active = False

        # Rotate goal
        if self._rot_active and self._rot_target_yaw is not None:
            _, _, _, cur_yaw = _get_world_pose_xy_yaw("/World/Spot/body")
            err = _wrap_pi(self._rot_target_yaw - cur_yaw)

            if abs(err) < self._rot_tol:
                self._rot_active = False
                self._rot_target_yaw = None
                wz = 0.0
            else:
                wz = max(-self._rot_max_wz, min(self._rot_max_wz, self._rot_kp * err))

        # Hold heading when idle to cancel drift
        if self._hold_heading and (not self._move_active) and (not self._rot_active):
            _, _, _, cur_yaw = _get_world_pose_xy_yaw("/World/Spot/body")
            if self._hold_yaw is None:
                self._hold_yaw = cur_yaw
            err = _wrap_pi(self._hold_yaw - cur_yaw)
            wz_hold = max(-self._hold_max_wz, min(self._hold_max_wz, self._hold_kp * err))
            self._base_cmd[:] = [0.0, 0.0, wz_hold]
        else:
            self._hold_yaw = None

        # Apply combined result
        self._base_cmd[:] = [vx, 0.0, wz]

        # If both finished, stop
        if (not self._move_active) and (not self._rot_active):
            self._base_cmd[:] = [0.0, 0.0, 0.0]

    # ----- Sensors -----
    def _init_camera_depth(self, cam_path: str):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(cam_path)
        if not prim or not prim.IsValid() or not prim.IsA(UsdGeom.Camera):
            log(f"Camera prim not found or not a UsdGeom.Camera: {cam_path}")
            return

        # Create render product + annotators
        self._rp = rep.create.render_product(Sdf.Path(cam_path), CAM_RES)

        # Note: in some builds "rgb" returns RGBA (you saw (H,W,4)), that's fine.
        self._rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self._depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")

        self._rgb_annot.attach(self._rp)
        self._depth_annot.attach(self._rp)

        log(f"Camera+Depth ready: {cam_path} @ {CAM_RES}")

    def _init_imu(self, imu_path: str):
        self._imu_iface = _sensor.acquire_imu_sensor_interface()
        log(f"IMU interface acquired; reading from {imu_path}")

    def _update_sensors(self):
        now = time.time()
        if (now - self._last_sensor_t) < (1.0 / SENSOR_HZ):
            return
        self._last_sensor_t = now

        #  DEPTH SUMMARY
        if self._depth_annot is not None:
            depth = self._depth_annot.get_data()  # (H,W) float32 meters

            # Robust min-distance in 3 horizontal bands (left/center/right)
            h, w = depth.shape
            band_y0, band_y1 = int(h * 0.45), int(h * 0.55)

            def finite_min(arr):
                arr = arr[np.isfinite(arr)]
                return float(arr.min()) if arr.size else float("inf")

            left = finite_min(depth[band_y0:band_y1, int(w*0.10):int(w*0.30)])
            front = finite_min(depth[band_y0:band_y1, int(w*0.45):int(w*0.55)])
            right = finite_min(depth[band_y0:band_y1, int(w*0.70):int(w*0.90)])

            # Store
            self.sensor_state["front_clear_m"] = front
            self.sensor_state["left_clear_m"] = left
            self.sensor_state["right_clear_m"] = right

        # ---- RGB (optional to store raw; usually you compress/summarize instead) ----
        # If you *really* want the raw frame, this returns (H,W,4) uint8 often.
        # rgb = self._rgb_annot.get_data()

        # ---- IMU ----
        if self._imu_iface is not None:
            r = self._imu_iface.get_sensor_reading(IMU_PATH, use_latest_data=True, read_gravity=True)
            if getattr(r, "is_valid", False):
                self.sensor_state["imu_lin_acc"] = [r.lin_acc_x, r.lin_acc_y, r.lin_acc_z]
                self.sensor_state["imu_ang_vel"] = [r.ang_vel_x, r.ang_vel_y, r.ang_vel_z]
                self.sensor_state["imu_orientation"] = r.orientation  # quaternion