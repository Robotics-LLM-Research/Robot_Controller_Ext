from .utils import log, _get_world_pose_xy_yaw


class TaskRuntime:
    """
    Object that extension calls every physics step
        - Exposes task-specific world state through API hooks
    """

    def __init__(self, target_path: str):
        self._target_path = target_path
        self._reset_needed = False

    # ---------- API hooks ----------
    def get_target(self):
        pose = _get_world_pose_xy_yaw(self._target_path, allow_missing=True)
        if pose is None:
            return None

        x, y, z, _yaw = pose
        return {
            "x": float(x),
            "y": float(y),
            "z": float(z),
        }

    def reset(self):
        """ Track reset requests initiated by task API """
        self._reset_needed = True
        return True

    # ---------- Wiring ----------
    def request_reset(self):
        self.reset()

    # ---------- Physics Loop ----------
    def step(self, dt: float):
        _ = float(dt)

        if self._reset_needed:
            self._reset_needed = False
            log("[TASK] reset requested", 2)
