import os
import requests
from google import genai
from google.genai import types

GEMINI_KEY = os.environ["GEMINI_API_KEY"]
SPOT_API = "http://127.0.0.1:8001"



# Function declaration
spot_move_function = {
    "name": "move_spot",
    "description": "Moves Spot by x amount of meters. If x is positive, Spot moves x meters forward. If x is negative, Spot moves x meters backward",
    "parameters" : {
        "type": "object",
        "properties": {
            "meters": {
                "type": "number",
                "description": "Specified distance of meters to move",
            }
        },
        "required": ["meters"],
    },
}

# Configure client and tools
client = genai.Client(api_key=GEMINI_KEY)
tools = types.Tool(function_declarations=[spot_move_function])
config = types.GenerateContentConfig(
    tools=[tools],
    # Force the model to call 'any' function, instead of chatting
    tool_config=types.ToolConfig(
        function_calling_config=types.FunctionCallingConfig(mode='ANY')
    )
)

# User prompt
input = input("Enter a command: ")
contents = [
    types.Content(
        role="user", parts=[types.Part(text=input)]
    )
]

# Send request with function declarations
response = client.models.generate_content(
    model="gemini-3-flash-preview",
    contents=contents,
    config=config,
)

# Check for a function call
if response.function_calls:
    fc = response.function_calls[0]

    print(f"\nFunction to call: {fc.name}")
    print(f"Arguments: {fc.args}")

    # Dispatch table maps
    if fc.name == "move_spot":
        meters = float(fc.args["meters"])
        r = requests.post(f"{SPOT_API}/move", params={"meters": meters}, timeout=5)
        print("API status:", r.status_code)
        print("API response:", r.json())
    else:
        print("Unknown function:", fc.name)
else:
    print("No function call found in the response.")
    print(response.text)