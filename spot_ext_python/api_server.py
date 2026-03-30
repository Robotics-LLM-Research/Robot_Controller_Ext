import queue
import threading

import uvicorn
from fastapi import FastAPI



def start_spot_api(cmd_q: "queue.Queue", host: str, port: int, get_status=None, get_sensors=None, get_frame=None):
    app = FastAPI()

    # ---------- ENDPOINTS ----------
    @app.get("/ping")
    def ping():
        return {"ok": True}
    
    @app.get("/status")
    def status():
        if get_status is None:
            return {"ok": False, "error": "status is not wired"}
        return {"ok": True, "status": get_status()}
    
    # ----- Locomotion -----
    # --- Base ---
    @app.post("/cmd_vel")
    def cmd_vel(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0):
        """
        Apply continuous velocity
        Linear (m/s) and angular (rads/s):
            - vx: pos -> forward || neg -> backward
            - vy: pos -> left || neg -> right
            - wz: pos -> counter-clockwise || neg -> clockwise
        """
        cmd_q.put(("cmd_vel", float(vx), float(vy), float(wz)))
        return {"queued": True}
    
    @app.post("/stop")
    def stop():
        """ Cancel any queued move/rotate and zero base velocity """
        cmd_q.put(("stop",))
        return {"queued": True}
  
    # --- Goal Based ---
    @app.post("/move")
    def move(meters: float = 1.0):
        """ Move forward(+) / backward(-) by meters """
        cmd_q.put(("move", float(meters)))
        return {"queued": True}
    
    @app.post("/rotate")
    def rotate(deg: float = 90.0):
        """ Rotate clockwise(+) / counter-clockwise(-) by degrees """
        cmd_q.put(("rotate", float(deg)))
        return {"queued": True}
    
    # --- Sensors ---   
    @app.get("/sensors")
    def sensors():
        if get_sensors is None:
            return {"ok": False, "error": "sensors are not wired"}
        return {"ok": True, "sensors": get_sensors()}

    @app.get("/frame")
    def frame():
        if get_frame is None:
            return {"ok": False, "error": "frame is not wired"}

        frame_data = get_frame()
        if frame_data is None:
            return {"ok": False, "error": "frame unavailable"}

        return {"ok": True, **frame_data}

    # ---------- SERVER ----------
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    return server, thread


def start_drone_api(cmd_q: "queue.Queue", host: str, port: int, get_status=None, get_sensors=None):
    app = FastAPI()

    # ---------- ENDPOINTS ----------
    @app.get("/ping")
    def ping():
        return {"ok": True}
    
    @app.get("/status")
    def status():
        if get_status is None:
            return {"ok": False, "error": "status is not wired"}
        return {"ok": True, "status": get_status()}
    
    # ----- Locomotion -----
    # --- Base ---
    @app.post("/cmd_vel")
    def cmd_vel(vx: float = 0.0, vy: float = 0.0, vz: float = 0.0, wz: float = 0.0):
        """
        Apply continuous velocity
        Linear (m/s) and angular (rads/s):
            - vx: pos -> forward || neg->backward
            - vy: pos -> left || neg -> right
            - vz: pos -> up || neg -> down
            - wz: pos -> counter-clockwise || neg -> clockwise
        """
        cmd_q.put(("cmd_vel", float(vx), float(vy), float(vz), float(wz)))
        return {"queued": True}
    
    @app.post("/stop")
    def stop():
        """ Cancel any queued move/rotate and zero base velocity """
        cmd_q.put(("stop",))
        return {"queued": True}
    
    # --- Goal Based ---
    @app.post("/move_fwd")
    def move_fwd(meters: float = 1.0):
        """ Move forward(+) / backward(-) by meters """
        cmd_q.put(("move_fwd", float(meters)))
        return {"queued": True}
    
    @app.post("/move_lat")
    def move_lateral(meters: float = 1.0):
        """ Move left(+) / right(-) by meters """
        cmd_q.put(("move_lat", float(meters)))
        return {"queued": True}
    
    @app.post("/raise_alt")
    def raise_alt(meters: float = 1.0):
        """ Change altitude up(+) / down(-) by meters """
        cmd_q.put(("raise_alt", float(meters)))
        return {"queued": True}
    
    @app.post("/rotate")
    def rotate(deg: float = 90.0):
        """ Rotate counter-clockwise(+) / clockwise(-) by degrees """
        cmd_q.put(("rotate", float(deg)))
        return {"queued": True}
    
    # --- Equipment ---
    @app.post("/look")
    def look(x: float = 0.0, y: float = 0.0):
        """ Move the drone on-board camera """
        cmd_q.put(("look", float(x), float(y)))
        return {"queued": True}
    
    # --- Sensors ---
    @app.get("/sensors")
    def sensors():
        if get_sensors is None:
            return {"ok": False, "error": "sensors are not wired"}
        return {"ok": True, "sensors": get_sensors()}
    
    # ---------- SERVER ----------
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    return server, thread