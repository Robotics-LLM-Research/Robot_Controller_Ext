import math
import queue
import numpy as np

import carb
import omni.usd
from pxr import UsdGeom
from omni.isaac.dynamic_control import _dynamic_control

from .utils import log
from .sensing import SensorSuite



# ----- Utils -----
def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

def _get_world_pose_xy_yaw(prim_path: str):
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
    """
    All locomotion state and creates base_cmd = [vx, vy, wz].
    Handles:
      - /cmd_vel (manual)
      - /move (distance goal)
      - /rotate (yaw goal)
      - /stop
    """

    def __init__(self, drone_body_path: str):
        self._drone_body_path = drone_body_path
        self.base_cmd = np.zeros(3)  # vx, vy, wz
        self._manual_cmd = np.zeros(3)

        # Move goal
        self._move_active = False
        self._move_remaining = 0.0   # meters remaining (+forward, -back)
        self._move_speed = 2.0       # m/s

        # Rotate goal
        self._rot_active = False
        self._rot_target_yaw = None
        self._rot_kp = 2.0
        self._rot_max_wz = 1.0
        self._rot_tol = math.radians(2.0)

    def reset(self):
        self.base_cmd[:] = [0.0, 0.0, 0.0]
        self._manual_cmd[:] = [0.0, 0.0, 0.0]
        self._move_active = False
        self._move_remaining = 0.0
        self._rot_active = False
        self._rot_target_yaw = None

    def handle_cmd(self, cmd):
        kind = cmd[0]

        if kind == "cmd_vel":
            _, vx, vy, wz = cmd
            self._move_active = False
            self._rot_active = False
            self._move_remaining = 0.0
            self._rot_target_yaw = None
            self._manual_cmd[:] = [float(vx), float(vy), float(wz)]
            log(f"[DRONE] cmd_vel set: vx={vx}, vy={vy}, wz={wz}")

        elif kind == "move":
            meters = float(cmd[1])
            self._manual_cmd[:] = [0.0, 0.0, 0.0]
            self._move_remaining += meters
            self._move_active = True
            log(f"[DRONE] move queued: {meters} m (remaining={self._move_remaining} m)")

        elif kind == "rotate":
            deg = float(cmd[1])
            self._manual_cmd[:] = [0.0, 0.0, 0.0]
            delta = math.radians(deg)
            _, _, _, cur_yaw = _get_world_pose_xy_yaw(self._droe_path)

            if self._rot_active and self._rot_target_yaw is not None:
                self._rot_target_yaw = _wrap_pi(self._rot_target_yaw + delta)
            else:
                self._rot_target_yaw = _wrap_pi(cur_yaw + delta)
                self._rot_active = True

            log(f"[DRONE] rotate target set: {deg} deg -> target_yaw={self._rot_target_yaw:.3f}")

        elif kind == "stop":
            self.reset()
            log("[DRONE] stop: cleared base cmd + goals")

        else:
            return False

        return True

    def update(self, dt: float):
        """
        Compute base_cmd for this step
        Priority:
          1) goals (move/rotate) if active
          2) manual cmd_vel if non-zero
        """
        dt = float(dt)

        # --- Goal mode ---
        if self._move_active or self._rot_active:
            vx = 0.0
            wz = 0.0

            # Move
            if self._move_active:
                if abs(self._move_remaining) > 1e-4:
                    direction = 1.0 if self._move_remaining > 0 else -1.0
                    vx = self._move_speed * direction

                    next_remaining = self._move_remaining - (vx * dt)
                    # if we crossed 0, clamp & stop
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
                _, _, _, cur_yaw = _get_world_pose_xy_yaw(self._drone_body_path)
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
            return self.base_cmd


class DroneRuntime:
    """
    Single object the Extension talks to
      - Reads cmd_vel = (vx, vy, vz, wz) from queue
      - Applies velocity to the Crazyflie root
    """

    def __init__(
        self,
        cmd_q: "queue.Queue",
        drone_path: str,
        cam_path: str,
        imu_path: str,
        cam_res=(640, 480),
        sensor_hz: float = 5.0,
    ):
        self.cmd_q = cmd_q
        self._drone_path = drone_path

        # Current command: vx, vy, vz, wz
        self._cmd = np.zeros(4, dtype=np.float32)

        # Sensors
        self.sensing = SensorSuite(
            cam_path=cam_path,
            imu_path=imu_path,
            cam_res=cam_res,
            sensor_hz=sensor_hz,
        )

        self._reset_needed = False

    # ----- API hooks -----
    def get_sensors(self):
        return self.sensing.get_sensors()

    # ----- Wiring -----
    def attach_drone(self, drone):
        self.drone = drone
        self.sensing.attach()
        log("[DRONE] runtime attached (drone + sensors)")

    def request_reset(self):
        self._reset_needed = True

    def step(self, dt: float):
        """
        Called from the PhysX step callback.
        """
        # Drain commands
        self._drain_cmd_queue()

        if self.drone is None:
            return

        # Reset requested (timeline stop -> play)
        if self._reset_needed:
            self._reset_needed = False
            self.motion.reset()
            return

        # Update sensing
        self.sensing.update()

        # Update locomotion + forward policy
        base_cmd = self.motion.update(dt)
        try:
            # TODO: Move the drone
        except Exception as e:
            log(f"[DRONE] __move the drone ig__ failed: {e}")

    def _drain_cmd_queue(self):
        while True:
            try:
                cmd = self.cmd_q.get_nowait()
            except queue.Empty:
                break

            # Route locomotion commands to motion controller
            handled = self.motion.handle_cmd(cmd)
            if not handled:
                log(f"[DRONE] unknown cmd: {cmd}")