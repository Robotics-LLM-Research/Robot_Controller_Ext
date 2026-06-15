# pyright: reportMissingImports=false
import numpy as np
from pxr import Gf
import re

# Stage
CANDIDATE_STAGE_ROOTS = ("/World", "/Root")

# Robots
RECOGNIZED_ROBOTS = ("Spot", "Drone")
ROBOT_NAME_PATTERN = rf"^({'|'.join(re.escape(name) for name in RECOGNIZED_ROBOTS)})(?:-(\d+))?$"
ROBOT_NAME_RE = re.compile(ROBOT_NAME_PATTERN)

SPOT_BASE_POSITION = np.array([0.0, 0.0, 0.8], dtype=np.float32)
SPOT_BASE_ROTATION_DEG = Gf.Vec3f(0.0, 0.0, 0.0)
DRONE_BASE_POSITION = Gf.Vec3d(0.5, -6.0, 3.5)
DRONE_BASE_ROTATION_DEG = Gf.Vec3f(0.0, 0.0, 90.0)

# Robot add-ons (under each robot root)
BODY_PRIM = "body"
FRONT_CAM_PRIM = "FrontCam"
SENSORS_PRIM = "Sensors"

# Task / environment
ENVIRONMENT_PRIM = "Environment"
TARGET_PRIM = "Target"

# HTTP API
HOST = "127.0.0.1"
BASE_PORT = 8001

# Sensors 
CAM_RES = (640, 480)
SENSOR_HZ = 5.0
