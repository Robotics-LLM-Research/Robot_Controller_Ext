import queue
import threading

import uvicorn
from fastapi import FastAPI



def start_spot_api(
    cmd_q: "queue.Queue", 
    host: str, 
    port: int, 
    get_status=None, 
    get_pose=None,
    get_sensors=None, 
    get_frame=None
):
    app = FastAPI()

    # ---------- ENDPOINTS ----------
    @app.get("/ping")
    def ping():
        return {"ok": True}
    
    @app.get("/status")
    def status():
        """ Get the status of Spot """
        if get_status is None:
            return {"ok": False, "error": "status is not wired"}
        
        status_data = get_status()
        if status_data is None:
            return {"ok": False, "error": "status unavailable"}

        return {"ok": True, "status": status_data}
    
    @app.get("/pose")
    def pose():
        """ Get the x, y, z, yaw of Spot """
        if get_pose is None:
            return {"ok": False, "error": "pose is not wired"}

        pose_data = get_pose()
        if pose_data is None:
            return {"ok": False, "error": "pose unavailable"}

        return {"ok": True, "pose": pose_data}
    
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
    
    # ----- Sensors -----   
    @app.get("/sensors")
    def sensors():
        """ Get the sensors and IMU data of Spot """
        if get_sensors is None:
            return {"ok": False, "error": "sensors are not wired"}

        sensors_data = get_sensors()
        if sensors_data is None:
            return {"ok": False, "error": "sensors unavailable"}

        return {"ok": True, "sensors": sensors_data}

    @app.get("/frame")
    def frame():
        """ Get the latest camera frame of Spot """
        if get_frame is None:
            return {"ok": False, "error": "frame is not wired"}

        frame_data = get_frame()
        if frame_data is None:
            return {"ok": False, "error": "frame unavailable"}

        return {"ok": True, "frame": frame_data}

    # ---------- SERVER ----------
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    return server, thread


def start_drone_api(
    cmd_q: "queue.Queue", 
    host: str, 
    port: int, 
    get_status=None, 
    get_sensors=None, 
    get_frame=None,
):
    app = FastAPI()

    # ---------- ENDPOINTS ----------
    @app.get("/ping")
    def ping():
        return {"ok": True}
    
    @app.get("/status")
    def status():
        if get_status is None:
            return {"ok": False, "error": "status is not wired"}

        status_data = get_status()
        if status_data is None:
            return {"ok": False, "error": "status unavailable"}

        return {"ok": True, "status": status_data}
    
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
    
    # ----- Sensors -----
    @app.get("/sensors")
    def sensors():
        """ Get the sensors and IMU data of Drone """
        if get_sensors is None:
            return {"ok": False, "error": "sensors are not wired"}

        sensors_data = get_sensors()
        if sensors_data is None:
            return {"ok": False, "error": "sensors unavailable"}

        return {"ok": True, "sensors": sensors_data}

    @app.get("/frame")
    def frame():
        """ Get the latest camera frame of Drone """
        if get_frame is None:
            return {"ok": False, "error": "frame is not wired"}

        frame_data = get_frame()
        if frame_data is None:
            return {"ok": False, "error": "frame unavailable"}

        return {"ok": True, "frame": frame_data}
    
    # ---------- SERVER ----------
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    return server, thread