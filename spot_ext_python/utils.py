import carb
import math



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