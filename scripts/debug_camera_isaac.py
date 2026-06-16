# Run this in Isaac Sim: Window -> Script Editor -> paste -> Run (while timeline is PLAYING).
# Tests camera capture step-by-step and prints which step fails.
#
# Edit CAM_PATH if your FrontCam lives elsewhere.

CAM_PATH = "/Root/Spot/body/FrontCam"
CAM_RES = (640, 480)

import traceback

import omni.replicator.core as rep
import omni.usd
from pxr import Sdf, UsdGeom


def step(name, fn):
    print(f"\n--- {name} ---")
    try:
        result = fn()
        print(f"OK: {name}")
        if result is not None:
            print(f"  -> {result}")
        return result
    except Exception as e:
        print(f"FAIL: {name}: {e!r}")
        traceback.print_exc()
        return None


stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath(CAM_PATH)
step("prim valid", lambda: prim and prim.IsValid())
step("prim is camera", lambda: prim.IsA(UsdGeom.Camera))

rp = step(
    "replicator.render_product",
    lambda: rep.create.render_product(Sdf.Path(CAM_PATH), CAM_RES),
)
if rp is None:
    raise SystemExit("render_product failed; stopping")

depth_annot = step(
    "get distance_to_camera annotator",
    lambda: rep.AnnotatorRegistry.get_annotator("distance_to_camera"),
)
if depth_annot:
    step("attach depth", lambda: depth_annot.attach(rp) or True)

rgb_annot = step("get rgb annotator", lambda: rep.AnnotatorRegistry.get_annotator("rgb"))
if rgb_annot:
    step("attach rgb (LdrColorSDhostPtr usually fails here)", lambda: rgb_annot.attach(rp) or True)

print("\n--- isaacsim.sensors.camera fallback ---")
try:
    from isaacsim.sensors.camera import Camera

    def make_cam():
        c = Camera(prim_path=CAM_PATH, resolution=CAM_RES, frequency=5.0)
        c.initialize(attach_rgb_annotator=False)
        return c

    cam = step("Camera()", make_cam)
    if cam:
        step("add_distance_to_camera_to_frame", cam.add_distance_to_camera_to_frame)
        for device in ("cpu", "cuda"):
            ok = step(f"attach_annotator rgb device={device}", lambda d=device: cam.attach_annotator("rgb", device=d) or True)
            if ok:
                break
        step("get_rgb sample", cam.get_rgb)
except Exception as e:
    print(f"isaac Camera path unavailable: {e!r}")
    traceback.print_exc()

print("\nDone. Paste this output when reporting camera issues.")
