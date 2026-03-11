import time
from types import ModuleType

from dotenv import load_dotenv
load_dotenv()

import gemini_agent, openai_agent
from tools import get_tool_declarations
from executor import execute_fc, get_status, get_sensors, emergency_stop

MAX_STEPS = 20
ROBOTS = ("Spot", "Drone")
TOOL_TO_ROBOT = {
    "move_spot": "Spot",
    "rotate_spot": "Spot",

    "move_forward_drone": "Drone",
    "move_lateral_drone": "Drone",
    "raise_altitude_drone": "Drone",
    "rotate_drone": "Drone",
    "look_drone": "Drone",
}

SYSTEM_PROMPT = (
    "You are the director for two simulated robots: Spot and Drone. "
    "You control them through function calls only. "
    "You will receive the latest world state, including status and sensors for both robots. "
    "Use that shared world state to decide which robot should act next. "
    "Spot and Drone may be used together to complete one mission. "
    "Call at most ONE function per turn. "
    "If the mission is not complete, call another function instead of asking for permission or merely describing the next step. "
    "Only respond with plain text when the mission is actually complete."
)



# ---------- State Management ----------
def initialize_state(user_mission):
    return {
        "mission"       : user_mission,
        "last_action"   : None,
        "last_result"   : None,
        "latest_world" : None, 
        "step_count"    : 0,
        "done"          : False,
        "log"           : [],
    }


def update_state(state, fc_name, fc_args, tool_result, world):
    state["last_action"] = {
        "name": fc_name,
        "args": fc_args,
    }
    state["last_result"] = tool_result
    state["latest_world"] = world
    state["step_count"] += 1
    return state


def add_log(state, message):
    state["log"].append(message)


def print_state(state):
    print("step_count: ", state["step_count"])
    print("\tlast_action: ", state["last_action"])
    print("\tlast_result: ", state["last_result"])
    print("\tdone: ", state["done"])

    if state["latest_world"] is not None:
        for robot_name, robot_data in state["latest_world"]["robots"].items():
            print(f"\t{robot_name} status:", robot_data["status"])


# ---------- Utils ----------
def select_agent(name):
    if name == "OpenAI":
        return openai_agent
    if name == "Gemini":
        return gemini_agent
    raise ValueError(f"Unknown AGENT: {name}")


def get_robot_for_tool(tool_name):
    if tool_name not in TOOL_TO_ROBOT:
        raise ValueError(f"Unknown tool name: {tool_name}")
    return TOOL_TO_ROBOT[tool_name]


def safe_json(response):
    payload = response.json()
    if not payload.get("ok", False):
        raise RuntimeError(payload)
    return payload


def collect_robot_payload(robot_name):
    status_payload = safe_json(get_status(robot_name))
    sensors_payload = safe_json(get_sensors(robot_name))

    return {
        "status": status_payload["status"],
        "sensors": sensors_payload["sensors"],
    }


def collect_world_payload():
    robots = {}
    for robot_name in ROBOTS:
        robots[robot_name] = collect_robot_payload(robot_name)

    return {
        "robots": robots
    }


def wait_until_robot_idle(robot_name, timeout_s=30, poll_s=0.2):
    """ Waits for the specific robot used by the latest tool call to finish. """
    start = time.time()
    saw_busy = False

    while time.time() - start < timeout_s:
        payload = safe_json(get_status(robot_name))
        status = payload["status"]

        if status.get("busy", False):
            saw_busy = True
        elif saw_busy:
            return status
        
        time.sleep(poll_s)

    # Perform emergency stop
    stop_response = emergency_stop(robot_name)
    print("Performed Emergency Stop.")
    if stop_response is not None:
            try:
                _ = stop_response.json()
            except Exception:
                pass

    raise TimeoutError("Robot did not finish in time")


# ---------- MAIN ----------
def run_director_loop(agent: ModuleType, client, model_context, user_mission):
    state = initialize_state(user_mission)
    add_log(state, f"Mission received: {user_mission}")

    contents = agent.create_initial_content(user_mission)

    # Give initial world
    initial_world = collect_world_payload()
    state["latest_world"] = initial_world
    contents.append(agent.create_observation_content(initial_world, label="Initial World State"))

    while state["step_count"] < MAX_STEPS:
        print_state(state)

        response = agent.ask_model(client, contents, model_context)
        agent.append_model_response(contents, response)

        # Execute function
        tool_call = agent.extract_tool_call(response)
        if tool_call is None:
            return agent.get_final_text(response)

        name = tool_call["name"]
        args = tool_call["args"]
        raw  = tool_call["raw"]

        acting_robot = get_robot_for_tool(name)
        print("Action taken: ", name, args)
        add_log(state, f"Director chose {name} for {acting_robot} with args={args}")

        tool_result = execute_fc(name, args)
        
        # Update status/sensors
        latest_status = wait_until_robot_idle(acting_robot)
        latest_world = collect_world_payload()
        contents.append(agent.create_tool_results(raw, tool_result, acting_robot, latest_status, latest_world))

        state = update_state(state, name, args, tool_result, latest_world)

    return("Max steps reached.")


def main():
    agent = select_agent("OpenAI")
    client = agent.create_client()
    model_context = agent.create_model_context(SYSTEM_PROMPT, get_tool_declarations())

    # Begin the mission
    user_mission = input("Enter mission: ")
    final_output = run_director_loop(agent, client, model_context, user_mission)
    print(final_output)


if __name__ == "__main__":
    main()