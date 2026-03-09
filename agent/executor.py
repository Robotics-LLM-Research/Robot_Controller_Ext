import requests

SPOT_API = "http://127.0.0.1:8001"
DRONE_API = "http://127.0.0.1:8002"


# --- Utils ---
def map_robot_to_api(cur_robot):
    if cur_robot == "Spot":
        return SPOT_API
    elif cur_robot == "Drone":
        return DRONE_API
    else:
        raise ValueError("No robot api for ", cur_robot)

# --- API GETS ---
def get_sensors(cur_robot):
    robot_api = map_robot_to_api(cur_robot)
    return requests.get(f"{robot_api}/sensors")

def get_status(cur_robot):
    robot_api = map_robot_to_api(cur_robot)
    return requests.get(f"{robot_api}/status")

# --- Command Handling ---
def execute_fc(fc):
    name = fc.name
    args = fc.args

    # Spot Commands
    if name == "move_spot":
        return requests.post(f"{SPOT_API}/move", params={"meters": float(args["meters"])}, timeout=5)
    
    if name == "rotate_spot":
        return requests.post(f"{SPOT_API}/rotate", params={"deg": float(args["degrees"])}, timeout=5)

    # Drone Commands
    if name == "move_forward_drone":
        return requests.post(f"{DRONE_API}/move_fwd", params={"meters": float(args["meters"])}, timeout=5)
    
    if name == "move_lateral_drone":
        return requests.post(f"{DRONE_API}/move_lat", params={"meters": float(args["meters"])}, timeout=5)
    
    if name == "raise_altitude_drone":
        return requests.post(f"{DRONE_API}/raise_alt", params={"meters": float(args["meters"])}, timeout=5)
    
    if name == "rotate_drone":
        return requests.post(f"{DRONE_API}/rotate", params={"deg": float(args["degrees"])}, timeout=5)
    
    if name == "look_drone":
        return requests.post(f"{DRONE_API}/look", params={"x": float(args["x"]), "y": float(args["y"])}, timeout=5)

    raise ValueError("Unknown function: ", name) 