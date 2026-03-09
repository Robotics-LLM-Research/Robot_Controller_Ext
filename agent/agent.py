import os
import json
import time

from google import genai
from google.genai import types, errors

from tools import get_tool_declarations
from executor import execute_fc, get_status, get_sensors

GEMINI_KEY = os.environ["GEMINI_API_KEY"]
MODEL = "gemini-2.5-flash"

MAX_STEPS = 10

SYSTEM_PROMPT = (
    "You control simulated robots through function calls only ."
    "Use the latest sensor readings. "
    "Choose one safe action at a time. "
    "If the mission is complete, do not call a function and instead respond with a short completion message."
)




# ---------- Model ----------
def create_client():
    return genai.Client(api_key=GEMINI_KEY)

def create_config(tool_declarations):
    tools = types.Tool(function_declarations=tool_declarations)
    return types.GenerateContentConfig (
        tools=[tools],
        system_instruction=SYSTEM_PROMPT,
        # Force the model to call 'any' function, instead of chatting
        tool_config=types.ToolConfig(
            function_calling_config=types.FunctionCallingConfig(mode='AUTO')
        )
    )

def ask_model(client, contents, config, retries=3):
    for attempt in range(retries):
        try:
            return client.models.generate_content(
                model=MODEL,
                contents=contents,
                config=config
            )
        except errors.ServerError as e: 
            if attempt == retries - 1:
                raise
            wait_s = 2 ** attempt
            print(f"Gemini unavailable, retrying in {wait_s}s...")
            time.sleep(wait_s)

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

def update_state(state, fc, tool_result, sensors):
    state["last_action"] = {
        "name": fc.name,
        "args": dict(fc.args),
    }
    state["last_result"] = tool_result
    state["latest_sensors"] = sensors
    return state

def print_state(state):
    print("step_count: ", state["step_count"])
    print("\tlast_action: ", state["last_action"])
    print("\tlast_result: ", state["last_result"])
    print("\tdone: ", state["done"])

# ---------- Content Management ----------
def create_initial_content(user_mission):
    return [
        types.Content(
            role="user", 
            parts=[types.Part(text=user_mission)]
        )
    ]

def build_function_callback_content(fc, tool_result, status, sensors):
    function_response = {
        "result": tool_result,
        "latest_status": status,
        "latest_sensors": sensors,
    }

    function_response_part = types.Part.from_function_response(
        name=fc.name,
        response=function_response,
    )

    return types.Content(
        role="user",
        parts=[function_response_part]
    )

# ---------- Main ----------
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

    raise TimeoutError("Robot did not finish in time")

def run_agent_loop(client, config, user_mission):
    state = initialize_state(user_mission)
    step_count = state["step_count"]

    contents = create_initial_content(user_mission)

    # Agent loop
    while step_count < MAX_STEPS:
        print_state(state)
        response = ask_model(client, contents, config)
        
        if not response.function_calls:
            return response.text

        # Model's response content
        contents.append(response.candidates[0].content)

        # Execute function
        fc = response.function_calls[0]
        print("Action taken: ", fc.name, fc.args)
        http_response = execute_fc(fc)
        tool_result = {
            "status_code": http_response.status_code,
            "body": http_response.json()
        }
        
        # Update status/sensors
        status = wait_until_idle()
        sensors = get_sensors("Spot").json()

        # Update Content
        tool_response_content = build_function_callback_content(fc, tool_result, status, sensors)
        contents.append(tool_response_content)

        # Update state
        step_count += 1
        state["step_count"] = step_count
        state["done"] = status["done"]
        state = update_state(state, fc, tool_result, sensors)

    return("Max steps reached.")

def main():
    # Inital config of model
    client = create_client()
    config = create_config(get_tool_declarations())

    # Begin the mission and agent loop
    user_mission = input("Enter mission: ")
    final_output = run_agent_loop(client, config, user_mission)

    print(final_output)


if __name__ == "__main__":
    main()