import carb
import math
import omni.usd
from pxr import UsdGeom



def log(msg: str, level: int):
    ext_msg = "[EXT] " + msg
    if level == -1:
        carb.log_warn(f"[DEBUG] {ext_msg}")
    if level == 1:
        carb.log_info(ext_msg)
    if level == 2:
        carb.log_warn(ext_msg)
    if level == 3:
        carb.log_error(ext_msg)

def _wrap_pi(a: float) -> float:
    """
    Normalize any anlge to the range (-π, π]. Used for "turn to heading"
    Returns: warped angle radians
    """
    return math.atan2(math.sin(a), math.cos(a))

def _get_world_pose_xy_yaw(prim_path: str, allow_missing: bool = False):
    """ Return (x, y, z, yaw) of prim in world frame """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if allow_missing and (prim is None or not prim.IsValid()):
        return None

    cache = UsdGeom.XformCache()
    m = cache.GetLocalToWorldTransform(prim)

    p = m.ExtractTranslation()
    x, y, z = float(p[0]), float(p[1]), float(p[2])

    r = m.ExtractRotationMatrix()
    yaw = math.atan2(float(r[1][0]), float(r[0][0]))
    return x, y, z, yaw