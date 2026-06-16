# pyright: reportMissingImports=false
import base64
import io
import math
import traceback
import numpy as np
import time

from isaacsim.sensors.physics import _sensor
import omni.replicator.core as rep
import omni.usd
from pxr import UsdGeom, Sdf

from .constants import CAM_RES, FRONT_CAM_PRIM, SENSORS_PRIM, SENSOR_HZ
from .utils import log, _to_numpy



class SensorSuite:
    """
    All sensing state. Extension calls:
      - attach() once after Play (validate prims, wire IMU)
      - init_camera() after render warmup (separate from attach)
      - update() each physics step
    """

    def __init__(
        self,
        cam_path: str,
        imu_path: str,
    ):
        self._cam_path = cam_path
        self._imu_path = imu_path

        self._cam_res = CAM_RES
        self._sensor_hz = float(SENSOR_HZ)

        # Replicator path
        self._rp = None
        self._rgb_annot = None
        self._depth_annot = None

        # Isaac Camera path
        self._camera = None
        self._rgb_enabled = False

        self._backend = None
        self._camera_ready = False
        self._depth_read_err_logged = False
        self._diag = self._empty_diag()

        self._imu_iface = None
        self._last_sensor_t = 0.0

        self._sensor_state = {
            "front_clear_m": None,
            "left_clear_m": None,
            "right_clear_m": None,
            "imu_lin_acc": [0.0, 0.0, 0.0],
            "imu_ang_vel": [0.0, 0.0, 0.0],
            "imu_orientation": None,
        }

    def _empty_diag(self) -> dict:
        return {
            "cam_path": self._cam_path,
            "ready": False,
            "backend": None,
            "steps": [],
            "last_error": None,
            "last_traceback": None,
            "prim_valid": None,
            "prim_is_camera": None,
            "render_product": None,
        }

    def _record_step(self, step: str, ok: bool, error: str | None = None, tb: str | None = None):
        entry = {"step": step, "ok": ok}
        if error:
            entry["error"] = error
        self._diag["steps"].append(entry)
        if not ok:
            self._diag["last_error"] = error
            self._diag["last_traceback"] = tb

    def get_camera_debug(self) -> dict:
        return dict(self._diag)

    # ----- Wiring -----
    def attach(self):
        """Validate prims and wire IMU. Camera init is init_camera()."""
        stage = omni.usd.get_context().get_stage()
        self._diag = self._empty_diag()

        if self._cam_path is not None:
            cam_prim = stage.GetPrimAtPath(self._cam_path)
            self._diag["prim_valid"] = bool(cam_prim and cam_prim.IsValid())
            self._diag["prim_is_camera"] = bool(
                cam_prim and cam_prim.IsValid() and cam_prim.IsA(UsdGeom.Camera)
            )
            if not self._diag["prim_valid"]:
                log(f"[SENSE] {FRONT_CAM_PRIM} not found at {self._cam_path}; camera not attached", 3)
                self._cam_path = None
            elif not self._diag["prim_is_camera"]:
                log(
                    f"[SENSE] {FRONT_CAM_PRIM} at {self._cam_path} is not a UsdGeom.Camera; camera not attached",
                    3,
                )
                self._cam_path = None

        if self._imu_path is not None:
            sensors_prim = stage.GetPrimAtPath(self._imu_path)
            if not sensors_prim or not sensors_prim.IsValid():
                log(f"[SENSE] {SENSORS_PRIM} prim not found at {self._imu_path}; IMU not attached", 3)
                self._imu_path = None

        if self._imu_path is not None:
            try:
                self._init_imu(self._imu_path)
            except Exception as e:
                log(f"[SENSE] IMU init failed: {e}", 3)
                self._imu_path = None

    def init_camera(self) -> bool:
        """Try camera backends after play + render warmup. Returns True if ready."""
        self._reset_camera_state()
        if self._cam_path is None:
            self._record_step("validate", False, "cam_path is None")
            return False

        log(f"[SENSE] init_camera start: {self._cam_path} @ {self._cam_res}", -1)

        if self._try_replicator():
            return True
        if self._try_isaac_camera():
            return True

        log(
            f"[SENSE] Camera init failed (all backends). Last: {self._diag.get('last_error')}",
            3,
        )
        return False

    def _reset_camera_state(self):
        self._rp = None
        self._rgb_annot = None
        self._depth_annot = None
        self._camera = None
        self._rgb_enabled = False
        self._backend = None
        self._camera_ready = False
        self._depth_read_err_logged = False
        self._diag["ready"] = False
        self._diag["backend"] = None
        self._diag["render_product"] = None
        self._diag["last_error"] = None
        self._diag["last_traceback"] = None
        self._diag["steps"] = []

    # ----- API getters -----
    def get_sensors(self):
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
        if not self._camera_ready:
            return None

        try:
            if self._backend == "replicator" and self._rgb_annot is not None:
                frame = self._rgb_annot.get_data()
            elif self._backend == "isaac_camera" and self._camera is not None and self._rgb_enabled:
                frame = self._camera.get_rgb()
            else:
                return None
        except Exception as e:
            log(f"[SENSE] rgb read failed: {e}", 3)
            return None

        rgb = _to_numpy(frame)
        if rgb is None or not hasattr(rgb, "shape") or len(rgb.shape) < 3:
            return None

        rgb = rgb[..., :3]
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
        self._update_camera_imu_throttled()

    def _update_camera_imu_throttled(self):
        now = time.time()
        if (now - self._last_sensor_t) < (1.0 / self._sensor_hz):
            return
        self._last_sensor_t = now

        if self._camera_ready:
            depth = self._read_depth()
            if depth is not None and hasattr(depth, "shape"):
                if len(depth.shape) == 3 and depth.shape[-1] == 1:
                    depth = depth[..., 0]
                if len(depth.shape) == 2:
                    h, w = depth.shape
                    band_y0, band_y1 = int(h * 0.45), int(h * 0.55)

                    def finite_min(arr):
                        arr = arr[np.isfinite(arr)]
                        return float(arr.min()) if arr.size else None

                    self._sensor_state["front_clear_m"] = finite_min(
                        depth[band_y0:band_y1, int(w * 0.45) : int(w * 0.55)]
                    )
                    self._sensor_state["left_clear_m"] = finite_min(
                        depth[band_y0:band_y1, int(w * 0.10) : int(w * 0.30)]
                    )
                    self._sensor_state["right_clear_m"] = finite_min(
                        depth[band_y0:band_y1, int(w * 0.70) : int(w * 0.90)]
                    )

        if self._imu_path is not None and self._imu_iface is not None:
            try:
                r = self._imu_iface.get_sensor_reading(
                    self._imu_path,
                    use_latest_data=True,
                    read_gravity=True,
                )
                if getattr(r, "is_valid", False):
                    self._sensor_state["imu_lin_acc"] = [r.lin_acc_x, r.lin_acc_y, r.lin_acc_z]
                    self._sensor_state["imu_ang_vel"] = [r.ang_vel_x, r.ang_vel_y, r.ang_vel_z]
                    self._sensor_state["imu_orientation"] = r.orientation
            except Exception as e:
                log(f"[SENSE] imu read failed: {e}", 3)

    def _read_depth(self):
        try:
            if self._backend == "replicator" and self._depth_annot is not None:
                return self._depth_annot.get_data()
            if self._backend == "isaac_camera" and self._camera is not None:
                frame_data = self._camera.get_current_frame()
                depth = frame_data.get("distance_to_camera")
                if depth is None:
                    depth = self._camera.get_depth()
                return _to_numpy(depth)
        except Exception as e:
            if not self._depth_read_err_logged:
                log(f"[SENSE] depth read failed: {e}", 3)
                self._depth_read_err_logged = True
        return None

    # ----- Camera backends -----
    def _mark_ready(self, backend: str):
        self._backend = backend
        self._camera_ready = True
        self._diag["ready"] = True
        self._diag["backend"] = backend
        log(f"[SENSE] Camera ready via {backend}: {self._cam_path} @ {self._cam_res}", 2)

    def _try_replicator(self) -> bool:
        cam_path = self._cam_path
        try:
            self._rp = rep.create.render_product(Sdf.Path(cam_path), self._cam_res)
            self._diag["render_product"] = str(self._rp)
            self._record_step("replicator.render_product", True)
        except Exception as e:
            self._record_step(
                "replicator.render_product",
                False,
                repr(e),
                traceback.format_exc(),
            )
            return False

        try:
            self._depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
            self._record_step("replicator.get_depth_annotator", True)
        except Exception as e:
            self._record_step(
                "replicator.get_depth_annotator",
                False,
                repr(e),
                traceback.format_exc(),
            )
            return False

        try:
            self._depth_annot.attach(self._rp)
            self._record_step("replicator.attach_depth", True)
        except Exception as e:
            self._record_step(
                "replicator.attach_depth",
                False,
                repr(e),
                traceback.format_exc(),
            )
            self._depth_annot = None
            return False

        try:
            self._rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            self._record_step("replicator.get_rgb_annotator", True)
        except Exception as e:
            self._record_step(
                "replicator.get_rgb_annotator",
                False,
                repr(e),
                traceback.format_exc(),
            )
            return False

        try:
            self._rgb_annot.attach(self._rp)
            self._record_step("replicator.attach_rgb", True)
        except Exception as e:
            self._record_step(
                "replicator.attach_rgb",
                False,
                repr(e),
                traceback.format_exc(),
            )
            log(f"[SENSE] replicator RGB attach failed: {e}", 3)
            self._rgb_annot = None
            # Depth-only is still useful for /sensors
            self._mark_ready("replicator")
            return True

        self._mark_ready("replicator")
        return True

    def _try_isaac_camera(self) -> bool:
        try:
            from isaacsim.sensors.camera import Camera
        except Exception as e:
            self._record_step("isaac_camera.import", False, repr(e), traceback.format_exc())
            return False

        cam_path = self._cam_path
        try:
            camera = Camera(
                prim_path=cam_path,
                resolution=self._cam_res,
                frequency=self._sensor_hz,
            )
            self._record_step("isaac_camera.construct", True)
        except Exception as e:
            self._record_step(
                "isaac_camera.construct",
                False,
                repr(e),
                traceback.format_exc(),
            )
            return False

        try:
            camera.initialize(attach_rgb_annotator=False)
            self._record_step("isaac_camera.initialize", True)
        except Exception as e:
            self._record_step(
                "isaac_camera.initialize",
                False,
                repr(e),
                traceback.format_exc(),
            )
            return False

        self._camera = camera

        try:
            camera.add_distance_to_camera_to_frame()
            self._record_step("isaac_camera.add_depth", True)
        except Exception as e:
            self._record_step(
                "isaac_camera.add_depth",
                False,
                repr(e),
                traceback.format_exc(),
            )

        for device in ("cpu", "cuda"):
            try:
                camera.attach_annotator("rgb", device=device)
                self._rgb_enabled = True
                self._record_step(f"isaac_camera.attach_rgb_{device}", True)
                break
            except Exception as e:
                self._record_step(
                    f"isaac_camera.attach_rgb_{device}",
                    False,
                    repr(e),
                    traceback.format_exc(),
                )

        if not self._rgb_enabled:
            log("[SENSE] isaac_camera RGB unavailable; depth-only if enabled", 3)

        self._mark_ready("isaac_camera")
        return True

    def _init_imu(self, imu_path: str):
        self._imu_iface = _sensor.acquire_imu_sensor_interface()
        log(f"[SENSE] IMU interface acquired; reading from {imu_path}", 2)
