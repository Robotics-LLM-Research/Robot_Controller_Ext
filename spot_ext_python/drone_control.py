import math
import queue
import numpy as np

import carb
import omni.usd
from pxr import UsdGeom, Gf
from omni.isaac.dynamic_control import _dynamic_control

from .utils import log, _wrap_pi
from .sensing import SensorSuite



# --- Camera Look ---
# Limits
LOOK_MAX_LEFT_DEG  = 70.0
LOOK_MAX_RIGHT_DEG = 70.0
LOOK_MAX_UP_DEG    = 45.0
LOOK_MAX_DOWN_DEG  = 45.0

# Neutral Trims
LOOK_TRIM_PITCH_DEG = 0.0
LOOK_TRIM_YAW_DEG   = 0.0
LOOK_TRIM_ROLL_DEG  = 0.0

# ---- Utils ----
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

def _clamp(v: float, lo: float, hi: float) -> float:
    """ Forces v to stay within [lo, hi] """
    return max(lo, min(hi, v))


class DroneLookController:
    """ Receives normalized (x,y) in [-1,1] and maps to yaw/pitch within limits """
    def __init__(
        self,
        cam_path: str,
        max_left_deg: float = LOOK_MAX_LEFT_DEG,
        max_right_deg: float = LOOK_MAX_RIGHT_DEG,
        max_up_deg: float = LOOK_MAX_UP_DEG,
        max_down_deg: float = LOOK_MAX_DOWN_DEG,
    ):
        self._cam_path = cam_path

        self._max_left = abs(float(max_left_deg))
        self._max_right = abs(float(max_right_deg))
        self._max_up = abs(float(max_up_deg))
        self._max_down = abs(float(max_down_deg))

        # Current offsets
        self._yaw_deg = 0.0    # + right, - left
        self._pitch_deg = 0.0  # + up, - down

        self._need_update = False
        self._prim = None
        self._xf = None
        self._rot_op = None

    def reset(self):
        self._yaw_deg = 0.0
        self._pitch_deg = 0.0
        self._need_update = True
    
    def attach(self) -> bool:
        """ Find camera prim and bind to the existing Rotate:look op """
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._cam_path)

        if prim is None or not prim.IsValid():
            log(f"[DRONE] Camera prim missing at {self._cam_path}", 3)
            self._prim = None
            self._xf = None
            self._rot_op = None
            return False

        self._prim = prim
        self._xf = UsdGeom.Xformable(prim)

        order_attr = prim.GetAttribute("xformOpOrder")
        order = list(order_attr.Get()) if order_attr and order_attr.IsValid() else []

        # Remove scale op
        scale_attr = prim.GetAttribute("xformOp:scale")
        if scale_attr and scale_attr.IsValid():
            s = scale_attr.Get()
            if abs(float(s[0]) - 1.0) < 1e-6 and abs(float(s[1]) - 1.0) < 1e-6 and abs(float(s[2]) - 1.0) < 1e-6:
                if "xformOp:scale" in order:
                    order = [t for t in order if t != "xformOp:scale"]
                    order_attr.Set(order)

        # Bind/create lookTrim (constant neutral)
        trim_attr = prim.GetAttribute("xformOp:rotateXYZ:lookTrim")
        if trim_attr and trim_attr.IsValid():
            self._trim_op = UsdGeom.XformOp(trim_attr)
        else:
            self._trim_op = self._xf.AddRotateXYZOp(opSuffix="lookTrim")
            trim_attr = self._trim_op.GetAttr()

        # Set trim ONCE (constant)
        trim_attr.Set(Gf.Vec3f(
            float(LOOK_TRIM_PITCH_DEG),
            float(LOOK_TRIM_YAW_DEG),
            float(LOOK_TRIM_ROLL_DEG),
        ))

        # Bind/create look (user-controlled offsets)
        look_attr = prim.GetAttribute("xformOp:rotateXYZ:look")
        if look_attr and look_attr.IsValid():
            self._rot_op = UsdGeom.XformOp(look_attr)
        else:
            self._rot_op = self._xf.AddRotateXYZOp(opSuffix="look")
            look_attr = self._rot_op.GetAttr()

        # Ensure both ops are in xformOpOrder 
        for name in ("xformOp:rotateXYZ:lookTrim", "xformOp:rotateXYZ:look"):
            if name not in order:
                order.append(name)
        order_attr.Set(order)

        look_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))

        self._need_update = False
        return True

    def handle_look(self, x: float, y: float):
        """
        x,y: normalized [-1,1]
          x: -1 left, +1 right
          y: -1 down, +1 up
        """
        x = _clamp(float(x), -1.0, 1.0)
        y = _clamp(float(y), -1.0, 1.0)

        # Map normalized to degrees with limits
        yaw = (x * self._max_right) if x >= 0.0 else (x * self._max_left)
        pitch = (y * self._max_up) if y >= 0.0 else (y * self._max_down)

        # Clamp again
        self._yaw_deg = _clamp(yaw, -self._max_left, self._max_right)
        self._pitch_deg = _clamp(pitch, -self._max_down, self._max_up)

        self._need_update = True
        log(f"[DRONE] look set: x={x:.2f}, y={y:.2f} -> yaw={self._yaw_deg:.1f}°, pitch={self._pitch_deg:.1f}°", 2)

    def apply_if_need_update(self):
        if not self._need_update:
            return
        
        if self._prim is None or (hasattr(self._prim, "IsValid") and not self._prim.IsValid()):
            if not self.attach():
                return
            
        if self._rot_op is None:
            if not self.attach():
                return

        # Yaw = X, Pitch = Y
        rot = Gf.Vec3f(
            float(self._pitch_deg),  # pitch on X
            float(self._yaw_deg),    # yaw on Y
            0.0
        )

        try:
            self._rot_op.Set(rot)
            self._need_update = False
        except Exception as e:
            log(f"[DRONE] look apply failed: {e} (will reattach)", 3)
            self._prim = None
            self._xf = None
            self._rot_op = None

    def has_pending_update(self) -> bool:
        return bool(self._need_update)


class DroneMotionController:
    """ Stores the "current desired motion" and interpret commands """

    def __init__(self):
        self.base_cmd = np.zeros(4, dtype=np.float32)
        self._manual_cmd = np.zeros(4, dtype=np.float32)

        # Linear goal
        self._fwd_active = False
        self._lat_active = False
        self._fwd_remaining = 0.0       # +forward, -backward
        self._lat_remaining = 0.0       # +left, -right
        self._move_speed = 4.0          # m/s

        # Rotate goal
        self._rot_active = False
        self._rot_target_yaw = None
        self._rot_pending_delta = 0.0   # radians waiting to be applied
        self._rot_kp = 4.0              # propotional gain (more or less rotation speed)
        self._rot_max_wz = 1.0
        self._rot_tol = math.radians(2.0)

        # Altitude goal
        self._alt_target_z = None
        self._alt_pending_delta = 0.0   # meters waiting to be applied
        self._alt_tol = 0.05            # meters

    def reset(self):
        self.base_cmd[:] = 0.0
        self._manual_cmd[:] = 0.0

        self._fwd_active = False
        self._lat_active = False
        self._fwd_remaining = 0.0
        self._lat_remaining = 0.0

        self._rot_active = False
        self._rot_target_yaw = None
        self._rot_pending_delta = 0.0

        self._alt_active = False
        self._alt_target_z = None
        self._alt_pending_delta = 0.0

    def handle_cmd(self, cmd):
        kind = cmd[0]

        if kind == "cmd_vel":
            _, vx, vy, vz, wz = cmd
           
            # Manual command overrides goals
            self._fwd_active = self._lat_active = False
            self._fwd_remaining = self._lat_remaining = 0.0
            self._rot_active = False
            self._rot_target_yaw = None
            self._rot_pending_delta = 0.0
            self._alt_active = False
            self._alt_pending_delta = 0.0

            self._manual_cmd[:] = [float(vx), float(vy), float(vz), float(wz)]
            log(f"[DRONE] cmd_vel set: vx={vx}, vy={vy}, vz={vz}, wz={wz}", 2)
        
        elif kind == "move_fwd":
            meters = float(cmd[1])
            self._manual_cmd[:] = 0.0
            self._fwd_remaining += meters
            self._fwd_active = True
            log(f"[DRONE] move_fwd queued: {meters} m (remaining={self._fwd_remaining} m)", 2)

        elif kind == "move_lat":
            meters = float(cmd[1])
            self._manual_cmd[:] = 0.0
            self._lat_remaining += meters
            self._lat_active = True
            log(f"[DRONE] move_lat queued: {meters} m (remaining={self._lat_remaining} m)", 2)

        elif kind == "rotate":
            deg = float(cmd[1])
            self._manual_cmd[:] = 0.0
            self._rot_pending_delta += math.radians(deg)
            log(f"[DRONE] rotate queued: {deg} deg (pending_delta_rad={self._rot_pending_delta:.3f})", 2)

        elif kind == "raise_alt":
            meters = float(cmd[1])
            self._manual_cmd[:] = 0.0
            self._alt_pending_delta += meters
            log(f"[DRONE] raise_alt queued: {meters} m (pending_delta={self._alt_pending_delta:.3f})", 2)
        
        elif kind == "stop":
            self.reset()
            log("[DRONE] stop", 2)

        else:
            return False
        
        return True
    
    def update(self, dt: float, cur_yaw: float, cur_z: float):
        """ Returns: (base_cmd, z_hold_target) """
        dt = float(dt)

        # Apply any pending rotate deltas using the current yaw
        if abs(self._rot_pending_delta) > 1e-9:
            if self._rot_target_yaw is None:
                self._rot_target_yaw = _wrap_pi(cur_yaw + self._rot_pending_delta)
                self._rot_active = True
            else:
                self._rot_target_yaw = _wrap_pi(self._rot_target_yaw + self._rot_pending_delta)
                self._rot_active = True
            self._rot_pending_delta = 0.0

        # Apply any pending altitude deltas using the current z
        if abs(self._alt_pending_delta) > 1e-9:
            if self._alt_target_z is None:
                self._alt_target_z = cur_z + self._alt_pending_delta
                self._alt_active = True
            else:
                self._alt_target_z = self._alt_target_z + self._alt_pending_delta
                self._alt_active = True
            self._alt_pending_delta = 0.0

        # --- Goal mode ---
        goal_active = self._fwd_active or self._lat_active or self._rot_active or self._alt_active
        if goal_active:
            vx = 0.0
            vy = 0.0
            wz = 0.0

            # Forward/back distance
            if self._fwd_active:
                if abs(self._fwd_remaining) > 1e-4:
                    direction = 1.0 if self._fwd_remaining > 0 else -1.0
                    vx = self._move_speed * direction
                    next_remaining = self._fwd_remaining - (vx * dt)
                    if (self._fwd_remaining > 0 and next_remaining <= 0) or (self._fwd_remaining < 0 and next_remaining >= 0):
                        self._fwd_remaining = 0.0
                        self._fwd_active = False
                        vx = 0.0
                    else:
                        self._fwd_remaining = next_remaining
                else:
                    self._fwd_remaining = 0.0
                    self._fwd_active = False

            # Left/right distance
            if self._lat_active:
                if abs(self._lat_remaining) > 1e-4:
                    direction = 1.0 if self._lat_remaining > 0 else -1.0
                    vy = self._move_speed * direction
                    next_remaining = self._lat_remaining - (vy * dt)
                    if (self._lat_remaining > 0 and next_remaining <= 0) or (self._lat_remaining < 0 and next_remaining >= 0):
                        self._lat_remaining = 0.0
                        self._lat_active = False
                        vy = 0.0
                    else:
                        self._lat_remaining = next_remaining
                else:
                    self._lat_remaining = 0.0
                    self._lat_active = False

            # Rotate
            if self._rot_active and self._rot_target_yaw is not None:
                err = _wrap_pi(self._rot_target_yaw - cur_yaw)
                if abs(err) < self._rot_tol:
                    self._rot_active = False
                    self._rot_target_yaw = None
                    wz = 0.0
                else:
                    wz = max(-self._rot_max_wz, min(self._rot_max_wz, self._rot_kp * err))

            # Altitude done check
            if self._alt_target_z is not None:
                if abs(self._alt_target_z - cur_z) < self._alt_tol:
                    self._alt_active = False
                    pass

            self.base_cmd[:] = [vx, vy, 0.0, wz]
            return self.base_cmd, self._alt_target_z

        # --- Manual cmd_vel mode ---
        if float(np.linalg.norm(self._manual_cmd)) > 1e-9:
            self.base_cmd[:] = self._manual_cmd
            return self.base_cmd, None

        self.base_cmd[:] = 0.0
        return self.base_cmd, None
    
    # ---------- Var Expose ----------
    def has_active_goal(self) -> bool:
        return self._fwd_active or self._lat_active or self._rot_active

    def has_manual_cmd(self) -> bool:
        return bool(np.linalg.norm(self._manual_cmd) > 1e-9)

    def active_goal_name(self):
        parts = []

        if self._fwd_active:
            parts.append("move_fwd")
        if self._lat_active:
            parts.append("move_lat")
        if self._rot_active:
            parts.append("rotate")
        if self._alt_active:
            parts.append("altitude")
        if self.has_manual_cmd():
            parts.append("cmd_vel")

        return "+".join(parts) if parts else None


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
        self._available = True
        self._missing_reason = None

        self.motion = DroneMotionController()
        self.sensing = SensorSuite(
            cam_path=cam_path,
            imu_path=imu_path,
            cam_res=cam_res,
            sensor_hz=sensor_hz,
        )
        self.look = DroneLookController(cam_path=cam_path)
        self.status = {
            "available": True,
            "busy": False,
            "idle": True,
            "queued_count": 0,
            "active_goal": None,
        }

    # ---------- API hooks ----------
    def get_status(self):
        return self.status
    
    def get_sensors(self):
        return self.sensing.get_sensors()

    def get_frame(self):
        return self.sensing.get_rgb_frame_jpeg_with_meta()

    # ---------- Wiring ----------
    def attach_drone(self):
        """ Connects sensors to prism """
        stage = omni.usd.get_context().get_stage()
        drone_prim = stage.GetPrimAtPath(self._drone_path)
        body_prim = stage.GetPrimAtPath(self._drone_body_path)

        if not drone_prim or not drone_prim.IsValid():
            self._available = False
            self._missing_reason = f"drone prim missing at {self._drone_path}"
            log(f"[DRONE] Disabled: {self._missing_reason}", 2)
            return
        if not body_prim or not body_prim.IsValid():
            self._available = False
            self._missing_reason = f"drone body prim missing at {self._drone_body_path}"
            log(f"[DRONE] Disabled: {self._missing_reason}", 2)
            return

        self._available = True
        self._missing_reason = None
        self.sensing.attach()
        self.look.attach()
        self.look.apply_if_need_update()

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
        return True

    # ---------- Physics Loop ----------
    def _refresh_status(self):
        queued_count = int(self.cmd_q.qsize())
        goal_active = bool(self.motion.has_active_goal())
        manual_active = bool(self.motion.has_manual_cmd())
        look_active = bool(self.look.has_pending_update())

        busy = self._available and ((queued_count > 0) or goal_active or manual_active or look_active)

        active_parts = []
        motion_name = self.motion.active_goal_name()
        if motion_name:
            active_parts.append(motion_name)
        if look_active:
            active_parts.append("look")

        self.status = {
            "available": self._available,
            "reason": self._missing_reason,
            "busy": busy,
            "idle": not busy,
            "queued_count": queued_count,
            "active_goal": "+".join(active_parts) if active_parts else None,
        }

    def _drain_cmd_queue(self):
        """ Empties queue and applies commands in order """
        while True:
            try:
                cmd = self.cmd_q.get_nowait()
            except queue.Empty:
                break

            kind = cmd[0]

            if kind == "look":
                _, x, y = cmd
                self.look.handle_look(x, y)
                continue

            handled = self.motion.handle_cmd(cmd)
            if not handled:
                log(f"[DRONE] unknown cmd: {cmd}", 3)

    def step(self, dt: float):
        """ Physics tick called from the PhysX step callback """
        dt = float(dt)
        self._drain_cmd_queue()
        if self._available:
            self.sensing.update()
            self.look.apply_if_need_update()
        
        # Handle timeline resets
        if self._reset_needed:
            self._reset_needed = False
            self.motion.reset()
            if hasattr(self, "_z_hold"):
                self._z_hold = None

            # Reset camera to look center
            self.look.reset()
            self.look.apply_if_need_update()
            self._refresh_status()
            return

        if not self._available:
            # Keep runtime idle when USD has no drone
            self.motion.reset()
            self._refresh_status()
            return

        if not self._ensure_handles():
            self._refresh_status()
            return

        # --- Control gains and clamps ---
        g = 9.81                # Gravity
        kp_z = 8.0              # Position error (corrects altitude error)
        kd_z = 4.0              # Damping on vertical vel (so no oscillation)

        kp_vxy = 2.0            # velocity tracking gain
        max_xy_accel = 2.0      # clamp on lateral acceleration

        #---- Read current state ---
        pose = self._dc.get_rigid_body_pose(self._body)                 # world position
        v = self._dc.get_rigid_body_linear_velocity(self._body)         # world linear vel 
        w = self._dc.get_rigid_body_angular_velocity(self._body)        # world angular vel

        z = float(pose.p.z)         # Hover altitude anchor
        vz = float(v.z)

        # Hold altitude
        if self._z_hold is None:
            self._z_hold = z

        # Get orientation from PhysX pose
        roll, pitch, yaw = _quat_to_rpy(pose.r)

        # Execute cmd from API 
        base_cmd, z_hold_target = self.motion.update(dt, yaw, z)

        if z_hold_target is not None:
            self._z_hold = float(z_hold_target)

        vx_b, vy_b, vz_cmd, wz_cmd = [float(x) for x in base_cmd]        

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

        self._refresh_status() 