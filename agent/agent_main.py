import os
import json
import time
from types import ModuleType

from dotenv import load_dotenv
load_dotenv()

import gemini_agent, openai_agent
from tools import get_tool_declarations
from executor import execute_fc, get_status, get_sensors, emergency_stop

MAX_STEPS = 20
SYSTEM_PROMPT = (
    "You control simulated robots through function calls only. "
    "Use the latest sensor readings. "
    "Choose one safe action at a time. "
    "The status field idle=True only means the previous action finished; it does NOT mean the mission is complete. "
    "If the mission is not complete, call another function instead of asking for permission or describing the next step. "
    "Only respond with plain text when the mission is actually complete."
)



# ---------- State Management ----------
def initialize_state(user_mission):
    return {
        "mission"       : user_mission,
        "last_action"   : None,
        "last_result"   : None,
        "latest_sensors": None, 
        "step_count"    : 0,
        "done"          : False
    }

def update_state(state, fc_name, fc_args, tool_result, sensors):
    state["last_action"] = {
        "name": fc_name,
        "args": fc_args,
    }
    state["last_result"] = tool_result
    state["latest_sensors"] = sensors
    state["step_count"] += 1
    return state

def print_state(state):
    print("step_count: ", state["step_count"])
    print("\tlast_action: ", state["last_action"])
    print("\tlast_result: ", state["last_result"])
    print("\tdone: ", state["done"])

# ---------- Utils ----------
def wait_until_idle(timeout_s=30, poll_s=0.2):
    start = time.time()
    saw_busy = False

    while time.time() - start < timeout_s:
        payload = get_status("Spot").json()

        if not payload.get("ok", False):
            raise RuntimeError(payload)
        
        status = payload["status"]

        if status["busy"]:
            saw_busy = True
        elif saw_busy:
            return status
        
        time.sleep(poll_s)

    # Perform emergency stop
    result = emergency_stop("Spot").json()
    print("Performed Emergency Stop.")
    if not result.get("ok", False):
            raise RuntimeError(payload)
    
    status = get_status("Spot").json()
    if status["status"]["busy"]:
            raise TimeoutError("Robot did not finish in time")

def select_agent(name):
    if name == "OpenAI":
        return openai_agent
    if name == "Gemini":
        return gemini_agent
    raise ValueError(f"Unknown AGENT: {name}")

# ---------- MAIN ----------
def run_agent_loop(agent: ModuleType, client, model_context, user_mission):
    state = initialize_state(user_mission)
    contents = agent.create_initial_content(user_mission)

    # Give initial sensors
    sensors = get_sensors("Spot").json()
    contents.append(agent.create_sensor_content(sensors, label="Initial Sensors"))

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

        print("Action taken: ", name, args)
        tool_result = execute_fc(name, args)
        
        # Update status/sensors
        status = wait_until_idle()
        sensors = get_sensors("Spot").json()
        contents.append(agent.create_tool_results(raw, tool_result, status, sensors))

        # Update state
        state["done"] = status["done"]
        state = update_state(state, name, args, tool_result, sensors)

    return("Max steps reached.")

def main():
    agent = select_agent("OpenAI")
    client = agent.create_client()
    model_context = agent.create_model_context(SYSTEM_PROMPT, get_tool_declarations())

    # Begin the mission
    user_mission = input("Enter mission: ")
    final_output = run_agent_loop(agent, client, model_context, user_mission)

    print(final_output)


if __name__ == "__main__":
    main()