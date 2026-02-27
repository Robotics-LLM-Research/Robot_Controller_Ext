import math
import time
import numpy as np

import omni.usd
import omni.replicator.core as rep

from pxr import UsdGeom, Sdf
from isaacsim.sensors.physics import _sensor

from .utils import log



# ----- Utils -----
def _get_world_pose_xy_yaw(prim_path: str):
    """
    Returns (x, y, z, yaw_rad) in world frame
    """
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


class SensorSuite:
    """
    All sensing state. Extension calls:
      - attach() once after Play (stage/prim paths exist)
      - update() each physics step
      - et_sensors() for the API
    """

    def __init__(
        self,
        cam_path: str,
        imu_path: str,
        cam_res=(640, 480),
        sensor_hz: float = 5.0,
    ):
        # Paths
        self._cam_path = cam_path
        self._imu_path = imu_path

        # Camera config
        self._cam_res = cam_res
        self._sensor_hz = float(sensor_hz)

        # Replicator annotators
        self._rp = None
        self._rgb_annot = None
        self._depth_annot = None

        # IMU
        self._imu_iface = None

        # Update throttling (wall-clock)
        self._last_sensor_t = 0.0

        # Public sensor returns
        self._sensor_state = {
            "front_clear_m": None,
            "left_clear_m": None,
            "right_clear_m": None,
            "imu_lin_acc": [0.0, 0.0, 0.0],
            "imu_ang_vel": [0.0, 0.0, 0.0],
            "imu_orientation": None,
        }

    # ----- Wiring -----
    def attach(self):
        """Call once after Play when camera + imu prims exist"""
        self._init_camera_depth(self._cam_path)
        if self._imu_path is not None:
            self._init_imu(self._imu_path)

    # ----- API getters -----
    def get_sensors(self):
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
        def safe(v):
            if isinstance(v, float):
                return v if math.isfinite(v) else None
            if isinstance(v, (list, tuple)):
                return [safe(x) for x in v]
            return v
        
        s = dict(self._sensor_state)
        ori = s.get("imu_orientation", None)
        if ori is not None and not isinstance(ori, (list, tuple)):
            try:
                s["imu_orientation"] = list(ori)
            except Exception:
                s["imu_orientation"] = None

        return {k: safe(v) for k, v in s.items()}

    # ----- Update loop -----
    def update(self):
        """
        Call every physics step
        - Camera/IMU summaries update at SENSOR_HZ
        """
        self._update_camera_imu_throttled()

    def _update_camera_imu_throttled(self):
        now = time.time()
        if (now - self._last_sensor_t) < (1.0 / self._sensor_hz):
            return
        self._last_sensor_t = now

        # Depth summary 
        if self._depth_annot is not None:
            depth = self._depth_annot.get_data()  # (H,W) float32 meters
            if depth is not None and hasattr(depth, "shape") and len(depth.shape) == 2:
                h, w = depth.shape
                band_y0, band_y1 = int(h * 0.45), int(h * 0.55)

                def finite_min(arr):
                    arr = arr[np.isfinite(arr)]
                    return float(arr.min()) if arr.size else None

                left = finite_min(depth[band_y0:band_y1, int(w * 0.10):int(w * 0.30)])
                front = finite_min(depth[band_y0:band_y1, int(w * 0.45):int(w * 0.55)])
                right = finite_min(depth[band_y0:band_y1, int(w * 0.70):int(w * 0.90)])

                self._sensor_state["front_clear_m"] = front
                self._sensor_state["left_clear_m"] = left
                self._sensor_state["right_clear_m"] = right

        # IMU
        if self._imu_path is not None and self._imu_iface is not None:
            try:
                r = self._imu_iface.get_sensor_reading(
                    self._imu_path,
                    use_latest_data=True,
                    read_gravity=True
                )
                if getattr(r, "is_valid", False):
                    self._sensor_state["imu_lin_acc"] = [r.lin_acc_x, r.lin_acc_y, r.lin_acc_z]
                    self._sensor_state["imu_ang_vel"] = [r.ang_vel_x, r.ang_vel_y, r.ang_vel_z]
                    self._sensor_state["imu_orientation"] = r.orientation  # quaternion
            except Exception as e:
                log(f"[SENSE] imu read failed: {e}", 3)

    # ----- Camera / IMU init -----
    def _init_camera_depth(self, cam_path: str):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(cam_path)
        if not prim or not prim.IsValid() or not prim.IsA(UsdGeom.Camera):
            log(f"[SENSE] Camera prim not found or not a UsdGeom.Camera: {cam_path}", 3)
            return

        self._rp = rep.create.render_product(Sdf.Path(cam_path), self._cam_res)
        self._rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self._depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")

        self._rgb_annot.attach(self._rp)
        self._depth_annot.attach(self._rp)

        log(f"[SENSE] Camera + Depth ready: {cam_path} @ {self._cam_res}", 2)

    def _init_imu(self, imu_path: str):
        self._imu_iface = _sensor.acquire_imu_sensor_interface()
        log(f"[SENSE] IMU interface acquired; reading from {imu_path}", 2)