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

    @app.post("/joint_delta")
    def joint_delta(joint: int = 0, delta: float = 0.2):
        """Queue intent. Isaac executes"""
        cmd_q.put(("joint_delta", int(joint), float(delta)))
        return {"queued": True}
    
    @app.post("/move")
    def move(meters: float = 1.0):
        cmd_q.put(("move", float(meters)))
        return {"queued": True}
    

    # ----- Server -----
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    
    def run():
        server.run()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()

    return server, thread