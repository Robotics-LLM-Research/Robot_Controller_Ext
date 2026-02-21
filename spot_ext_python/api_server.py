import threading
import queue

from fastapi import FastAPI
import uvicorn

def start_api(cmd_q: "queue.Queue", host, port):
    app = FastAPI()

    # ----- Endpoints -----
    @app.get("/ping")
    def ping():
        return {"ok": True}
  
    @app.post("/move")
    def move(meters: float = 1.0):
        cmd_q.put(("move", float(meters)))
        return {"queued": True}
    
    @app.post("/cmd_vel")
    def cmd_vel(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0):
        # vx, vy linear in m/s, wz angular in rad/s
        cmd_q.put(("cmd_vel", float(vx), float(vy), float(wz)))
        return {"queued": True}

    # ----- Server -----
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    
    def run():
        server.run()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()

    return server, thread