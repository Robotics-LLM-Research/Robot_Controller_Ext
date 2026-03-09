import math
import queue

import omni.usd
import numpy as np
from pxr import UsdGeom

from .utils import log, _wrap_pi
from .sensing import SensorSuite



# ----- Utils -----
def _get_world_pose_xy_yaw(prim_path: str):
    """ Return (x, y, z, yaw) of prim in world frame """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)

    cache = UsdGeom.XformCache()
    m = cache.GetLocalToWorldTransform(prim)

    p = m.ExtractTranslation()
    x, y, z = float(p[0]), float(p[1]), float(p[2])

    r = m.ExtractRotationMatrix()
    yaw = math.atan2(float(r[1][0]), float(r[0][0]))
    return x, y, z, yaw


class MotionController:
    """ Stores the "current desired motion" and interpret commands """

    def __init__(self, spot_body_path: str):
        self._spot_body_path = spot_body_path
        self.base_cmd = np.zeros(3)
        self._manual_cmd = np.zeros(3)

        # Move goal
        self._move_active = False
        self._move_remaining = 0.0   # +forward, -backward
        self._move_speed = 2.0       # m/s

        # Rotate goal
        self._rot_active = False
        self._rot_target_yaw = None
        self._rot_kp = 2.0
        self._rot_max_wz = 1.0
        self._rot_tol = math.radians(2.0)

        # Drift-cancel / heading hold when fully idle
        self._hold_heading = True
        self._hold_yaw = None
        self._hold_kp = 0.5
        self._hold_max_wz = 0.2

    def reset(self):
        self.base_cmd[:] = [0.0, 0.0, 0.0]
        self._manual_cmd[:] = [0.0, 0.0, 0.0]

        self._move_active = False
        self._move_remaining = 0.0
        self._rot_active = False
        self._rot_target_yaw = None
        self._hold_yaw = None

    def handle_cmd(self, cmd):
        kind = cmd[0]

        if kind == "cmd_vel":
            _, vx, vy, wz = cmd

            # Manual command overrides goals
            self._hold_yaw = None
            self._move_active = False
            self._rot_active = False
            self._move_remaining = 0.0
            self._rot_target_yaw = None
            
            self._manual_cmd[:] = [float(vx), float(vy), float(wz)]
            log(f"[SPOT] cmd_vel set: vx={vx}, vy={vy}, wz={wz}", 1)

        elif kind == "move":
            self._hold_yaw = None

            meters = float(cmd[1])
            self._manual_cmd[:] = [0.0, 0.0, 0.0]
            self._move_remaining += meters
            self._move_active = True
            log(f"[SPOT] move queued: {meters} m (remaining={self._move_remaining} m)", 1)

        elif kind == "rotate":
            self._hold_yaw = None

            deg = float(cmd[1])
            self._manual_cmd[:] = [0.0, 0.0, 0.0]
            delta = math.radians(deg)
            _, _, _, cur_yaw = _get_world_pose_xy_yaw(self._spot_body_path)

            if self._rot_active and self._rot_target_yaw is not None:
                self._rot_target_yaw = _wrap_pi(self._rot_target_yaw + delta)
            else:
                self._rot_target_yaw = _wrap_pi(cur_yaw + delta)
                self._rot_active = True

            log(f"[SPOT] rotate target set: {deg} deg -> target_yaw={self._rot_target_yaw:.3f}", 1)

        elif kind == "stop":
            self.reset()
            log("[SPOT] stop: cleared base cmd + goals", 1)

        else:
            return False

        return True

    def update(self, dt: float):
        """ Returns: base_cmd """
        dt = float(dt)

        # --- Goal mode ---
        if self._move_active or self._rot_active:
            self._hold_yaw = None
            vx = 0.0
            wz = 0.0

            # Move
            if self._move_active:
                if abs(self._move_remaining) > 1e-4:
                    direction = 1.0 if self._move_remaining > 0 else -1.0
                    vx = self._move_speed * direction

                    next_remaining = self._move_remaining - (vx * dt)
                    if (self._move_remaining > 0 and next_remaining <= 0) or (self._move_remaining < 0 and next_remaining >= 0):
                        self._move_remaining = 0.0
                        self._move_active = False
                        vx = 0.0
                    else:
                        self._move_remaining = next_remaining
                else:
                    self._move_remaining = 0.0
                    self._move_active = False

            # Rotate
            if self._rot_active and self._rot_target_yaw is not None:
                _, _, _, cur_yaw = _get_world_pose_xy_yaw(self._spot_body_path)
                err = _wrap_pi(self._rot_target_yaw - cur_yaw)

                if abs(err) < self._rot_tol:
                    self._rot_active = False
                    self._rot_target_yaw = None
                    wz = 0.0
                else:
                    wz = max(-self._rot_max_wz, min(self._rot_max_wz, self._rot_kp * err))

            # If everything finished, zero output
            if (not self._move_active) and (not self._rot_active):
                self.base_cmd[:] = [0.0, 0.0, 0.0]
            else:
                self.base_cmd[:] = [vx, 0.0, wz]

            return self.base_cmd

        # --- Manual cmd_vel mode ---
        if np.linalg.norm(self._manual_cmd) > 1e-9:
            self.base_cmd[:] = self._manual_cmd
            self._hold_yaw = None
            return self.base_cmd

        # --- Idle: hold heading to cancel drift ---
        if self._hold_heading:
            _, _, _, cur_yaw = _get_world_pose_xy_yaw(self._spot_body_path)
            if self._hold_yaw is None:
                self._hold_yaw = cur_yaw
            err = _wrap_pi(self._hold_yaw - cur_yaw)
            wz_hold = max(-self._hold_max_wz, min(self._hold_max_wz, self._hold_kp * err))
            self.base_cmd[:] = [0.0, 0.0, wz_hold]
            return self.base_cmd

        self.base_cmd[:] = [0.0, 0.0, 0.0]
        return self.base_cmd
    
    # ---------- Var Expose ----------
    def has_active_goal(self) -> bool:
        return self._move_active or self._rot_active

    def has_manual_cmd(self) -> bool:
        return bool(np.linalg.norm(self._manual_cmd) > 1e-9)

    def active_goal_name(self):
        parts = []

        if self._move_active:
            parts.append("move")
        if self._rot_active:
            parts.append("rotate")
        if self.has_manual_cmd():
            parts.append("cmd_vel")

        return "+".join(parts) if parts else None


class SpotRuntime:
    """
    Object that extension calls every physics step
        - Drains queued commands into the motion controller
        - Updates sensors
        - Applies velocities to drone rigidbody
    """

    def __init__(
        self,
        cmd_q: "queue.Queue",
        spot_body_path: str,
        cam_path: str,
        imu_path: str,
        cam_res=(640, 480),
        sensor_hz: float = 5.0,
    ):
        self.cmd_q = cmd_q
        self.spot = None

        self._reset_needed = False
        self._policy_inited = False
        self._warmup_left = 0

        self.motion = MotionController(spot_body_path=spot_body_path)
        self.sensing = SensorSuite(
            cam_path=cam_path,
            imu_path=imu_path,
            cam_res=cam_res,
            sensor_hz=sensor_hz,
        )
        self.status = {
            "busy": False,
            "done": True,
            "queued_count": 0,
            "active_goal": None,
        }

    # ---------- API hooks ----------
    def get_status(self):
        return self.status

    def get_sensors(self):
        return self.sensing.get_sensors()

    # ---------- Wiring ----------
    def attach_spot(self, spot):
        """ Connects sensors to prism """
        self.spot = spot
        self.sensing.attach()

    def request_reset(self):
        """ Next setp will reset step """
        self._reset_needed = True

    # ---------- Physics Loop ----------
    def _refresh_status(self):
        queued_count = int(self.cmd_q.qsize())
        goal_active = bool(self.motion.has_active_goal())
        manual_active = bool(self.motion.has_manual_cmd())

        busy = (queued_count > 0) or goal_active or manual_active

        self.status = {
            "busy": busy,
            "done": not busy,
            "queued_count": queued_count,
            "active_goal": self.motion.active_goal_name(),
        }

    def _drain_cmd_queue(self):
        """ Empties queue and applies commands in order """
        while True:
            try:
                cmd = self.cmd_q.get_nowait()
            except queue.Empty:
                break

            handled = self.motion.handle_cmd(cmd)
            if not handled:
                log(f"[SPOT] unknown cmd: {cmd}", 3)

    def step(self, dt: float):
        """ Physics tick called from the PhysX step callback """
        dt = float(dt)
        self._drain_cmd_queue()
        self.sensing.update()

        if self.spot is None:
            self._refresh_status()
            return

        # Handle timeline resets
        if self._reset_needed:
            self._reset_needed = False
            self.motion.reset()
            self._policy_inited = False
            self._warmup_left = 60
            self._refresh_status()
            return

        # Initialize policy once
        if not self._policy_inited:
            try:
                self.spot.initialize()
                self._policy_inited = True
                self._warmup_left = 60
            except Exception as e:
                log(f"[SPOT] spot.initialize not ready yet: {e}", 3)
            self._refresh_status()
            return

        # Warmup delay before commanding motion
        if self._warmup_left > 0:
            self._warmup_left -= 1
            self._refresh_status()
            return

        # Update locomotion + forward policy
        base_cmd = self.motion.update(dt)
        base_cmd = np.array(base_cmd, dtype=np.float32, copy=True)
        base_cmd[2] *= -1.0 # Fix wz sign

        try:
            self.spot.forward(float(dt), base_cmd)
        except Exception as e:
            log(f"[SPOT] spot.forward failed: {e}", 3)

        self._refresh_status()