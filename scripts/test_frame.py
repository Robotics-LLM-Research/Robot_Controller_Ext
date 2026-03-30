import base64
import json
import os
import time
from urllib import request
from urllib.error import HTTPError, URLError

SPOT_API = "http://127.0.0.1:8001"
FRAME_ENDPOINT = "/frame"
OUTDIR = "scripts/output"
REQUEST_TIMEOUT_S = 5.0



def fetch_frame(url: str, timeout: float):
    with request.urlopen(url, timeout=timeout) as resp:
        raw = resp.read()
    return json.loads(raw.decode("utf-8"))

def main():
    url = f"{SPOT_API}{FRAME_ENDPOINT}"

    try:
        payload = fetch_frame(url=url, timeout=REQUEST_TIMEOUT_S)
    except (HTTPError, URLError, TimeoutError) as e:
        print(f"[ERROR] request failed: {e}")
        raise SystemExit(1)
    except json.JSONDecodeError as e:
        print(f"[ERROR] non-JSON response from /frame: {e}")
        raise SystemExit(1)

    if not payload.get("ok", False):
        print(f"[ERROR] API returned failure: {payload}")
        raise SystemExit(1)

    b64 = payload.get("image_base64")
    if not b64:
        print("[ERROR] missing image_base64 in response")
        raise SystemExit(1)

    try:
        jpg_bytes = base64.b64decode(b64)
    except Exception as e:
        print(f"[ERROR] base64 decode failed: {e}")
        raise SystemExit(1)

    ts = payload.get("timestamp", time.time())
    stamp = str(ts).replace(".", "_")
    base_name = f"spot_frame_{stamp}"

    os.makedirs(OUTDIR, exist_ok=True)
    jpg_path = os.path.join(OUTDIR, f"{base_name}.jpg")
    meta_path = os.path.join(OUTDIR, f"{base_name}.json")

    with open(jpg_path, "wb") as f:
        f.write(jpg_bytes)

    meta = {
        "timestamp": payload.get("timestamp"),
        "frame_name": payload.get("frame_name"),
        "width": payload.get("width"),
        "height": payload.get("height"),
        "size": payload.get("size"),
        "format": payload.get("format"),
    }
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    print(f"[OK] saved image: {jpg_path}")
    print(f"[OK] saved metadata: {meta_path}")


if __name__ == "__main__":
    main()
