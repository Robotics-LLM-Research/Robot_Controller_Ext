import carb
import math
import re
import omni.usd
from pxr import UsdGeom, UsdPhysics



# ----- Stage -----
CANDIDATE_STAGE_ROOTS = ("/World", "/Root")
_ROBOT_NAME_RE = re.compile(r"^(Spot|Drone)(?:-(\d+))?$")

def get_stage_root(stage):
    """
    Return the top-level simulation scope for the open stage, if found.

    Args:
        stage: Usd.Stage.

    Resolution order:
      1. Stage default prim (defaultPrim metadata)
      2. Parent of the first PhysicsScene prim
      3. Known Isaac conventions (/World, /Root)
    """
    if stage is None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None

    if stage.HasDefaultPrim():
        prim = stage.GetDefaultPrim()
        if prim and prim.IsValid():
            return str(prim.GetPath())

    for prim in stage.Traverse():
        if not prim.IsValid():
            continue
        if prim.IsA(UsdPhysics.Scene):
            parent = prim.GetParent()
            if parent and parent.IsValid() and parent.GetPath() != "/":
                return str(parent.GetPath())

    for path in CANDIDATE_STAGE_ROOTS:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            return path

    return None


def discover_robots(stage, stage_root):
    """
    Find robot prims directly under stage_root.

    Matches prim names: Spot, Drone, Spot-1, Spot-2, Drone-1, ...

    Args:
        stage: Usd.Stage.
        stage_root: Resolved stage root path (e.g. "/World").

    Returns:
        List of dicts in sibling order, each with:
          - kind: "spot" or "drone"
          - name: prim name (e.g. "Spot-2")
          - path: full prim path
          - index: suffix number for Spot-1/Drone-2, else None for bare Spot/Drone
    """
    if stage is None or not stage_root:
        return []

    root_prim = stage.GetPrimAtPath(stage_root)
    if not root_prim or not root_prim.IsValid():
        return []

    found = []
    for child in root_prim.GetChildren():
        if not child.IsValid():
            continue

        name = child.GetName()
        match = _ROBOT_NAME_RE.match(name)
        if not match:
            continue

        index_str = match.group(2)
        found.append({
            "kind": match.group(1).lower(),
            "name": name,
            "path": str(child.GetPath()),
            "index": int(index_str) if index_str else None,
        })

    return found


# ----- Other -----
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
