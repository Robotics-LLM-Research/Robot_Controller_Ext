import queue
import threading

import uvicorn
from fastapi import FastAPI



def start_spot_api(cmd_q: "queue.Queue", host: str, port: int, get_sensors=None):
    app = FastAPI()

    # ---------- ENDPOINTS ----------
    @app.get("/ping")
    def ping():
        return {"ok": True}
    
    # --- Locomotion ---
    @app.post("/cmd_vel")
    def cmd_vel(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0):
        """
        Applies continuous velocity
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
  
    @app.post("/move")
    def move(meters: float = 1.0):
        """ Move forward(+) / backward(-) a certain distance in meters """
        cmd_q.put(("move", float(meters)))
        return {"queued": True}
    
    @app.post("/rotate")
    def rotate(deg: float = 90.0):
        """ Rotate counter-clockwise(+) / clockwise(-) a certain angle of degrees """
        cmd_q.put(("rotate", float(deg)))
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


def start_drone_api(cmd_q: "queue.Queue", host: str, port: int, get_sensors=None):
    app = FastAPI()

    # ---------- ENDPOINTS ----------
    @app.get("/ping")
    def ping():
        return {"ok": True}
    
    # --- Locomotion ---
    @app.post("/cmd_vel")
    def cmd_vel(vx: float = 0.0, vy: float = 0.0, vz: float = 0.0, wz: float = 0.0):
        """
        Applies continuous velocity
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
    
    # TODO
    @app.post("/move")
    def move(meters: float = 1.0):
        """ Move forward(+) / backward(-) a certain distance in meters """
        cmd_q.put(("move", float(meters)))
        return {"queued": True}
    
    # TODO
    @app.post("/rotate")
    def rotate(deg: float = 90.0):
        """ Rotate counter-clockwise(+) / clockwise(-) a certain angle of degrees """
        cmd_q.put(("rotate", float(deg)))
        return {"queued": True}
    
    # TODO
    @app.post("/look")
    def look(x: float = 0.0, y: float = 0.0):
        """
        Move the drone on-board camera
        """
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