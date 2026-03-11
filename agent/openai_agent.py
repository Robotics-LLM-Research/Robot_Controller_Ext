import json
from openai import OpenAI

MODEL = "gpt-5-mini"


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
        parallel_tool_calls=False,
        input=contents,
    )


def get_operator_message(response):
    """
    Returns any assistant-facing plain text from the model.
    Used for operator logs before a tool call or at mission completion.
    """
    text = response.output_text.strip()
    return text if text else None


# ----- Content -----
def create_initial_content(user_mission):
    return [{"role": "user", "content": user_mission}]


def create_observation_content(observation, label="Observation"):
    return {
        "role": "user",
        "content": (
            f"{label}:\n"
            + json.dumps(observation, indent=2)
            + "\nUse this shared world state to decide the next single action."
        )
    }


def append_model_response(contents, response):
    contents.extend(response.output)


def create_tool_results(raw_call, tool_result, acting_robot, latest_status, world_state):
    return {
        "type": "function_call_output",
        "call_id": raw_call.call_id,
        "output": json.dumps({
            "acting_robot": acting_robot,
            "result": tool_result,
            "latest_status": latest_status,
            "world_state": world_state,
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