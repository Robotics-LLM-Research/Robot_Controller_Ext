import math
import queue
import numpy as np

import carb
from omni.isaac.dynamic_control import _dynamic_control

from .utils import log
from .sensing import SensorSuite



# ----- Utils -----
def _wrap_pi(a: float) -> float:
    """
    Normalize any anlge to the range (-π, π]. Used for "turn to heading"
    Returns: warped angle radians
    """
    return math.atan2(math.sin(a), math.cos(a))

def _quat_to_rpy(q) -> tuple[float, float, float]:
    """ Convert a quaternion (x,y,z,w) into roll/pitch/yaw in radians """
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)

    # Rotation matrix from quaternion
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r10 = 2.0 * (x * y + w * z)

    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    yaw = math.atan2(r10, r00)
    pitch = math.atan2(-r20, math.sqrt(r21 * r21 + r22 * r22))
    roll = math.atan2(r21, r22)
    return roll, pitch, yaw

class DroneMotionController:
    """ Stores the "current desired motion" and interpret commands """

    def __init__(self):
        """ Creates base command [vx, vy, vz, wz] as 0s """
        self.base_cmd = np.zeros(4, dtype=np.float32)

    def update(self):
        """ Returns current stored command """
        return self.base_cmd

    def reset(self):
        """ Sets all commands to 0 """
        self.base_cmd[:] = 0.0

    def handle_cmd(self, cmd):
        """
        Commands:
        - cmd_vel: continuous motion
        - stop: stops all commands
        """
        kind = cmd[0]
        if kind == "cmd_vel":
            _, vx, vy, vz, wz = cmd
            self.base_cmd[:] = [vx, vy, vz, wz]
            log(f"[DRONE] cmd_vel set: vx={vx}, vy={vy}, vz={vz}, wz={wz}", 1)
            return True
        
        if kind == "stop":
            self.reset()
            log("[DRONE] stop", 1)
            return True
        
        return False


class DroneRuntime:
    """
    Object that extension calls every physics step
        - Drains queued commands into the motion controller
        - Updates sensors
        - Applies velocities to drone rigidbody
    """

    def __init__(
        self,
        cmd_q: "queue.Queue",
        drone_path: str,
        drone_body_path: str,
        cam_path: str,
        imu_path: str | None,
        cam_res=(640, 480),
        sensor_hz: float = 5.0,
    ):
        self.cmd_q = cmd_q
        self._drone_path = drone_path
        self._drone_body_path = drone_body_path
        self._reset_needed = False

        self.motion = DroneMotionController()
        self.sensing = SensorSuite(
            cam_path=cam_path,
            imu_path=imu_path,
            cam_res=cam_res,
            sensor_hz=sensor_hz,
        )

    # ---------- API hooks ----------
    def get_sensors(self):
        return self.sensing.get_sensors()

    # ---------- Wiring ----------
    def attach_drone(self):
        """ Connects sensors to prism """
        self.sensing.attach()

    def request_reset(self):
        """ Next setp will reset step """
        self._reset_needed = True
  
    def _ensure_handles(self):
        """
        Ensures we have valid body and dc
        Inits:
            - dc: Dynamic Control interface
            - body: rigid body handle for the drone prim path
        """
        if hasattr(self, "_dc") is False:
            self._dc = _dynamic_control.acquire_dynamic_control_interface()     # Dynamic Control interface
            self._body = 0
            self._z_hold = None                                                 # Hover altitude

        if self._body != 0:
            return True

        self._body = self._dc.get_rigid_body(self._drone_body_path)
        if self._body == 0:
            log(f"[DRONE] Could not get rigid body at {self._drone_body_path}", 3)
            return False
        
        # Zero out all current linear/angular velocity
        self._dc.set_rigid_body_linear_velocity(self._body, carb.Float3(0.0, 0.0, 0.0))
        self._dc.set_rigid_body_angular_velocity(self._body, carb.Float3(0.0, 0.0, 0.0))

        ##################################################################################### DEBUG PRINT
        log(f"[DRONE] Using rigid body handle {self._body} for {self._drone_body_path}", -1) 
        #####################################################################################
        return True

    # ---------- Physics Loop ----------
    def _drain_cmd_queue(self):
        """ Empties queue and applies commands in order """
        while True:
            try:
                cmd = self.cmd_q.get_nowait()
            except queue.Empty:
                break

            handled = self.motion.handle_cmd(cmd)
            if not handled:
                log(f"[DRONE] unknown cmd: {cmd}", 3)

    def step(self):
        """ Physics tick called from the PhysX step callback """
        self._drain_cmd_queue()
        self.sensing.update()
        
        # Handle timeline resets
        if self._reset_needed:
            self._reset_needed = False
            self.motion.reset()
            if hasattr(self, "_z_hold"):
                self._z_hold = None
            return

        if not self._ensure_handles():
            return

        # --- Control gains and clamps ---
        g = 9.81                # Gravity
        kp_z = 8.0              # Position error (corrects altitude error)
        kd_z = 4.0              # Damping on vertical vel (so no oscillation)

        kp_vxy = 2.0            # velocity tracking gain
        max_xy_accel = 2.0          # clamp on lateral acceleration

        #---- Read current state ---
        pose = self._dc.get_rigid_body_pose(self._body)                 # world position
        v = self._dc.get_rigid_body_linear_velocity(self._body)         # world linear vel 
        w = self._dc.get_rigid_body_angular_velocity(self._body)        # world angular vel

        z = float(pose.p.z)         # Hover altitude anchor
        vz = float(v.z)

        # Hold altitude
        if self._z_hold is None:
            self._z_hold = z

        # cmd from API 
        vx_b, vy_b, vz_cmd, wz_cmd = [float(x) for x in self.motion.update()]

        # orientation from PhysX pose
        roll, pitch, yaw = _quat_to_rpy(pose.r)

        # body -> world velocity transform using yaw
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        vx_w = cos_yaw * vx_b - sin_yaw * vy_b
        vy_w = sin_yaw * vx_b + cos_yaw * vy_b

        # --- Leveling roll/pitch ---
        level_gain_rp = 4.0
        max_roll_pitch_rate = 2.0
        roll_rate_cmd = max(-max_roll_pitch_rate, min(max_roll_pitch_rate, -level_gain_rp * roll))
        pitch_rate_cmd = max(-max_roll_pitch_rate, min(max_roll_pitch_rate, -level_gain_rp * pitch))

        self._dc.set_rigid_body_angular_velocity(
            self._body,
            carb.Float3(roll_rate_cmd, pitch_rate_cmd, float(wz_cmd))
        )

        # --- Vertical force ---
        az = kp_z * (self._z_hold - z) + kd_z * (0.0 - vz) + (2.0 * vz_cmd)

        # --- Lateral force ---
        ax = kp_vxy * (vx_w - float(v.x))
        ay = kp_vxy * (vy_w - float(v.y))

        # clamp
        ax = max(-max_xy_accel, min(max_xy_accel, ax))
        ay = max(-max_xy_accel, min(max_xy_accel, ay))

        m = 0.03
        try:
            props = self._dc.get_rigid_body_properties(self._body)
            if hasattr(props, "mass") and props.mass > 0:
                m = float(props.mass)
        except Exception:
            pass

        # Total force
        Fx = m * ax
        Fy = m * ay
        Fz = max(0.0, m * (g + az))

        self._dc.apply_body_force(
            self._body,
            carb.Float3(Fx, Fy, Fz),
            carb.Float3(float(pose.p.x), float(pose.p.y), float(pose.p.z)),
            True # False = body frame, True = world frame
        )