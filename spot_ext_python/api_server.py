import threading
import queue

from fastapi import FastAPI
import uvicorn


def start_api(cmd_q: "queue.Queue", host: str, port: int, get_lidar=None):
    app = FastAPI()

    # ----- Endpoints -----
    @app.get("/ping")
    def ping():
        return {"ok": True}
    
    @app.post("/cmd_vel")
    def cmd_vel(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0):
        """
        vx, vy linear in m/s, wz angular in rad/s
        """
        cmd_q.put(("cmd_vel", float(vx), float(vy), float(wz)))
        return {"queued": True}
  
    @app.post("/move")
    def move(meters: float = 1.0):
        """
        Move forward/backward a certain distance in meters
        """
        cmd_q.put(("move", float(meters)))
        return {"queued": True}
    
    @app.post("/rotate")
    def rotate(deg: float = 90.0):
        cmd_q.put(("rotate", float(deg)))
        return {"queued": True}
    
    @app.post("/stop")
    def stop():
        """
        Cancel any queued move/rotate and zero base velocity
        """
        cmd_q.put(("stop",))
        return {"queued": True}
    
    @app.post("/lidar_config")
    def lidar_config(yaw_deg: float = 0.0, max_dist: float = 10.0):
        cmd_q.put(("lidar_cfg", float(yaw_deg), float(max_dist)))
        return {"queued": True}

    @app.get("/lidar")
    def lidar():
        if get_lidar is None:
            return {"ok": False, "error": "lidar not wired"}
        dist, hit = get_lidar()
        return {"ok": True, "distance_m": dist, "hit": hit}

    # ----- Server -----
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    
    def run():
        server.run()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()

    return server, thread