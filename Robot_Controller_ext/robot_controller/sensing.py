# pyright: reportMissingImports=false
import base64
import io
import math
import numpy as np
import time

from isaacsim.sensors.physics import _sensor
import omni.replicator.core as rep
import omni.usd
from pxr import UsdGeom, Sdf

from .constants import CAM_RES, FRONT_CAM_PRIM, SENSORS_PRIM, SENSOR_HZ
from .utils import log



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
    ):
        # Paths
        self._cam_path = cam_path
        self._imu_path = imu_path

        # Camera config
        self._cam_res = CAM_RES
        self._sensor_hz = float(SENSOR_HZ)

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
        """Call once after Play when FrontCam and Sensors prims exist under body."""
        stage = omni.usd.get_context().get_stage()

        if self._cam_path is not None:
            cam_prim = stage.GetPrimAtPath(self._cam_path)
            if not cam_prim or not cam_prim.IsValid():
                log(f"[SENSE] {FRONT_CAM_PRIM} not found at {self._cam_path}; camera not attached", 3)
                self._cam_path = None
            elif not cam_prim.IsA(UsdGeom.Camera):
                log(
                    f"[SENSE] {FRONT_CAM_PRIM} at {self._cam_path} is not a UsdGeom.Camera; camera not attached",
                    3,
                )
                self._cam_path = None

        if self._cam_path is not None:
            self._init_camera_depth(self._cam_path)
        else:
            log("[SENSE] Camera not attached", 2)

        if self._imu_path is not None:
            sensors_prim = stage.GetPrimAtPath(self._imu_path)
            if not sensors_prim or not sensors_prim.IsValid():
                log(f"[SENSE] {SENSORS_PRIM} prim not found at {self._imu_path}; IMU not attached", 3)
                self._imu_path = None

        if self._imu_path is not None:
            self._init_imu(self._imu_path)
        else:
            log("[SENSE] IMU not attached", 2)

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

    def get_rgb_frame_jpeg_with_meta(self):
        """
        Returns latest camera RGB frame as base64 JPEG plus metadata.
        Returns None when camera is not ready or frame unavailable.
        """
        if self._rgb_annot is None:
            return None

        try:
            frame = self._rgb_annot.get_data()
        except Exception as e:
            log(f"[SENSE] rgb read failed: {e}", 3)
            return None

        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 3:
            return None

        # Replicator RGB data can be HxWx4; keep RGB only
        rgb = frame[..., :3]
        if rgb.dtype != np.uint8:
            try:
                rgb = np.clip(rgb, 0, 255).astype(np.uint8)
            except Exception:
                return None

        h, w = rgb.shape[:2]
        try:
            from PIL import Image
            img = Image.fromarray(rgb, mode="RGB")
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=90)
            jpeg_bytes = buf.getvalue()
        except Exception as e:
            log(f"[SENSE] jpeg encode failed: {e}", 3)
            return None

        return {
            "timestamp": time.time(),
            "frame_name": self._cam_path,
            "width": int(w),
            "height": int(h),
            "format": "jpeg",
            "image_base64": base64.b64encode(jpeg_bytes).decode("ascii"),
        }

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
        self._rp = rep.create.render_product(Sdf.Path(cam_path), self._cam_res)
        self._rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self._depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")

        self._rgb_annot.attach(self._rp)
        self._depth_annot.attach(self._rp)

        log(f"[SENSE] Camera + Depth ready: {cam_path} @ {self._cam_res}", 2)

    def _init_imu(self, imu_path: str):
        self._imu_iface = _sensor.acquire_imu_sensor_interface()
        log(f"[SENSE] IMU interface acquired; reading from {imu_path}", 2)