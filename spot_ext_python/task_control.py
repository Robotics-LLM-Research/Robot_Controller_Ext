import math

import omni.usd
from pxr import UsdGeom

from .utils import log



# ----- Utils -----
def _get_world_pose(prim_path: str):
    """ Return (x, y, z, yaw) of prim in world frame """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if prim is None or not prim.IsValid():
        return None

    cache = UsdGeom.XformCache()
    m = cache.GetLocalToWorldTransform(prim)

    p = m.ExtractTranslation()
    x, y, z = float(p[0]), float(p[1]), float(p[2])

    r = m.ExtractRotationMatrix()
    yaw = math.atan2(float(r[1][0]), float(r[0][0]))
    return x, y, z, yaw


class TaskRuntime:
    """
    Object that extension calls every physics step
        - Exposes task-specific world state through API hooks
    """

    def __init__(self, target_path: str):
        self._target_path = target_path
        self._reset_needed = False
        self.status = {
            "ready": True,
        }

    # ---------- API hooks ----------
    def get_status(self):
        return self.status

    def get_target(self):
        pose = _get_world_pose(self._target_path)
        if pose is None:
            return None

        x, y, z, yaw = pose
        return {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "yaw_rad": float(yaw),
        }

    def reset(self):
        """ Track reset requests initiated by task API """
        self._reset_needed = True
        return True

    # ---------- Wiring ----------
    def request_reset(self):
        """ Next step will perform reset """
        self._reset_needed = True

    # ---------- Physics Loop ----------
    def step(self, dt: float):
        _ = float(dt)

        if self._reset_needed:
            self._reset_needed = False
            log("[TASK] reset requested", 2)
