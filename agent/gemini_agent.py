import time
from google import genai
from google.genai import types, errors

MODEL = "gemini-2.5-flash"


# ----- Model -----
def create_client():
    return genai.Client()

def create_model_context(system_prompt, tools):
    gemini_tools = types.Tool(function_declarations=tools)
    config = types.GenerateContentConfig(
        tools=[gemini_tools],
        system_instruction=system_prompt,
        tool_config=types.ToolConfig(
            function_calling_config=types.FunctionCallingConfig(mode="AUTO")
        )
    )
    return {"config": config}

def ask_model(client, contents, model_context, retries=3):
    for attempt in range(retries):
        try:
            return client.models.generate_content(
                model=MODEL,
                contents=contents,
                config=model_context["config"],
            )
        except errors.ServerError as e:
            if attempt == retries - 1:
                raise
            wait_s = 2 ** attempt
            print(f"Gemini unavailable, retrying in {wait_s}s...")
            time.sleep(wait_s)

# ----- Content -----
def create_initial_content(user_mission):
    return [
        types.Content(role="user", parts=[types.Part(text=user_mission)])
    ]

def append_model_response(contents, response):
    contents.append(response.candidates[0].content)

def create_tool_result(raw_call, tool_result, status, sensors):
    function_response = {
        "result": tool_result,
        "latest_status": status,
        "latest_sensors": sensors,
    }

    part = types.Part.from_function_response(
        name=raw_call.name,
        response=function_response,
    )

    return types.Content(role="user", parts=[part])

# ----- Conversion -----
def extract_tool_call(response):
    if not response.function_calls:
        return None
    fc = response.function_calls[0]
    return {
        "name": fc.name,
        "args": dict(fc.args),
        "raw": fc,
    }

def get_final_text(response):
    return response.text