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
def execute_fc(name, args):

    # Spot Commands
    if name == "move_spot":
        response = requests.post(f"{SPOT_API}/move", params={"meters": float(args["meters"])}, timeout=5)
    
    elif name == "rotate_spot":
        response = requests.post(f"{SPOT_API}/rotate", params={"deg": float(args["degrees"])}, timeout=5)

    # Drone Commands
    elif name == "move_forward_drone":
        response = requests.post(f"{DRONE_API}/move_fwd", params={"meters": float(args["meters"])}, timeout=5)
    
    elif name == "move_lateral_drone":
        response = requests.post(f"{DRONE_API}/move_lat", params={"meters": float(args["meters"])}, timeout=5)
    
    elif name == "raise_altitude_drone":
        response = requests.post(f"{DRONE_API}/raise_alt", params={"meters": float(args["meters"])}, timeout=5)
    
    elif name == "rotate_drone":
        response = requests.post(f"{DRONE_API}/rotate", params={"deg": float(args["degrees"])}, timeout=5)
    
    elif name == "look_drone":
        response = requests.post(f"{DRONE_API}/look", params={"x": float(args["x"]), "y": float(args["y"])}, timeout=5)

    else:
        raise ValueError("Unknown function: ", name) 
    
    return {
        "status_code": response.status_code,
        "body": response.json()
    }