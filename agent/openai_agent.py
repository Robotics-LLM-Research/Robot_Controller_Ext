import json
from openai import OpenAI

MODEL = "gpt-5"


# ----- Model -----
def create_client():
    return OpenAI()

def create_model_context(system_prompt, tools):
    return {
        "system_prompt": system_prompt,
        "tools": tools,
    }

def ask_model(client, contents, model_context):
    return client.responses.create(
        model=MODEL,
        instructions=model_context["system_prompt"],
        tools=model_context["tools"],
        input=contents,
    )

# ----- Content -----
def create_initial_content(user_mission):
    return [{"role": "user", "content": user_mission}]

def append_model_response(contents, response):
    contents.extend(response.output)

def create_tool_results(raw_call, tool_result, status, sensors):
    return {
        "type": "function_call_output",
        "call_id": raw_call.call_id,
        "output": json.dumps({
            "result": tool_result,
            "latest_status": status,
            "latest_sensors": sensors,
        }),
    }

# ----- Conversion -----
def extract_tool_call(response):
    for item in response.output:
        if item.type == "function_call":
            return {
                "name": item.name,
                "args": json.loads(item.arguments),
                "raw": item,
            }
    return None

def get_final_text(response):
    return response.output_text