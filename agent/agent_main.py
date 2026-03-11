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
    "Drone is the only robot with environment perception for planning around obstacles. "
    "Spot should be treated as blind for obstacle analysis. "
    "If the mission involves a wall, obstacle, going around something, or choosing a path, use Drone observations first before commanding Spot through the obstacle area. "
    "Call at most ONE function per turn. "
    "Before each function call, include a short operator-facing sentence that explains what you are about to do. "
    "Only respond with plain text and no function call when the mission is actually complete."
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
    drone = collect_robot_payload("Drone")
    spot_status = safe_json(get_status("Spot"))["status"]

    return {
        "robots": {
            "Spot": {
                "status": spot_status,
                "sensors": None,
            },
            "Drone": drone,
        }
    }


def wait_until_robot_idle(robot_name, timeout_s=30, poll_s=0.2, idle_polls_needed=2):
    """ Waits for the specific robot used by the latest tool call to finish. """
    start = time.time()
    idle_streak = 0

    while time.time() - start < timeout_s:
        payload = safe_json(get_status(robot_name))
        status = payload["status"]

        busy = bool(status.get("busy", False))
        queued_count = int(status.get("queued_count", 0))

        if (not busy) and queued_count == 0:
            idle_streak += 1
            if idle_streak >= idle_polls_needed:
                return status
        else:
            idle_streak = 0

        time.sleep(poll_s)

    emergency_stop(robot_name)
    print("Performed Emergency Stop.")
    raise TimeoutError("Robot did not finish in time")


# ---------- MAIN ----------
def initialize_mission(agent: ModuleType, user_mission: str):
    state = initialize_state(user_mission)
    add_log(state, f"Mission received: {user_mission}")

    contents = agent.create_initial_content(user_mission)

    initial_world = collect_world_payload()
    state["latest_world"] = initial_world

    contents.append(
        agent.create_observation_content(
            initial_world,
            label="Initial World State"
        )
    )

    return state, contents


def run_director_loop(agent: ModuleType, client, model_context, user_mission):
    state, contents = initialize_mission(agent, user_mission)
    print(f"\nDirector: Got it. {user_mission}\n")

    while state["step_count"] < MAX_STEPS:
        response = agent.ask_model(client, contents, model_context)
        agent.append_model_response(contents, response)

        operator_msg = agent.get_operator_message(response)
        if operator_msg:
            print(f"Director: {operator_msg}")

        # Execute function
        tool_call = agent.extract_tool_call(response)
        if tool_call is None:
            return agent.get_final_text(response)

        name = tool_call["name"]
        args = tool_call["args"]
        raw  = tool_call["raw"]

        acting_robot = get_robot_for_tool(name)
        add_log(state, f"Director chose {name} for {acting_robot} with args={args}")

        print(f"Director: Executing {name} on {acting_robot} with args={args}")
        tool_result = execute_fc(name, args)
        
        # Update status/sensors
        latest_status = wait_until_robot_idle(acting_robot)
        latest_world = collect_world_payload()

        contents.append(agent.create_tool_results(raw, tool_result, acting_robot, latest_status, latest_world))
        state = update_state(state, name, args, tool_result, latest_world)

    return "Mission stopped because max steps were reached."


def main():
    agent = select_agent("OpenAI")
    client = agent.create_client()
    model_context = agent.create_model_context(SYSTEM_PROMPT, get_tool_declarations())

    print("Director console ready.")

    while True:
        user_mission = input("\nEnter mission (or 'q' to quit): ").strip()

        if not user_mission:
            continue
        if user_mission.lower() in {"q", "quit", "exit"}:
            print("Exiting director console.")
            break

        try:
            final_output = run_director_loop(agent, client, model_context, user_mission)
            print(f"\nDirector: {final_output}")
        except Exception as e:
            print(f"\nDirector: Mission failed with error: {e}")

        again = input("\nStart a new mission? [y/n]: ").strip().lower()
        if again not in {"y", "yes"}:
            print("Exiting director console.")
            break


if __name__ == "__main__":
    main()