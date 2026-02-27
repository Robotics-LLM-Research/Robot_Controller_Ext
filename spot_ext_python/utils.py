import carb



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