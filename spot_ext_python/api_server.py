import threading
import queue

from fastapi import FastAPI
import uvicorn


def start_api(cmd_q: "queue.Queue", host: str, port: int):
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

    # ----- Server -----
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    
    def run():
        server.run()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()

    return server, thread