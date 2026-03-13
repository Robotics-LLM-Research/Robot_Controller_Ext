import re
import json
import math

FLOAT_TOLERANCE = 1e-6



# ---------- Parsing ----------
def parse_tool_call(output_text: str) -> dict | None:
    clean_text = output_text.replace("<|im_end|>", "").strip()

    match = re.search(
        r"<tool_call>\s*(\{.*?\})\s*</tool_call>",
        clean_text, 
        re.DOTALL
    )
    if not match:
        return None
    
    json_text = match.group(1)

    try:
        parsed_call = json.loads(json_text)
    except json.JSONDecodeError as e:
        return None
    
    return parsed_call


# ---------- Schema Validation ----------
def _build_tool_schema_map(tool_declarations: list) -> dict:
    tool_schema_map = {}

    for tool in tool_declarations:
        function_data = tool["function"]
        tool_name = function_data["name"]
        tool_parameters = function_data["parameters"]

        tool_schema_map[tool_name] = tool_parameters

    return tool_schema_map


def validate_tool_call(response_call: dict | None, tool_declarations: list) -> dict:
    if response_call is None:
        return {
            "is_valid": False,
            "reason": "No parseable tool call found.",
        }

    if not isinstance(response_call, dict):
        return {
            "is_valid": False,
            "reason": "Parsed tool call is not a dictionary.",
        }

    if "name" not in response_call:
        return {
            "is_valid": False,
            "reason": "Missing 'name' field.",
        }

    if "arguments" not in response_call:
        return {
            "is_valid": False,
            "reason": "Missing 'arguments' field.",
        }

    tool_name = response_call["name"]
    arguments = response_call["arguments"]

    if not isinstance(arguments, dict):
        return {
            "is_valid": False,
            "reason": "'arguments' must be a dictionary.",
        }

    tool_schema_map = _build_tool_schema_map(tool_declarations)

    if tool_name not in tool_schema_map:
        return {
            "is_valid": False,
            "reason": f"Unknown tool name: {tool_name}",
        }

    parameters_schema = tool_schema_map[tool_name]
    properties = parameters_schema.get("properties", {})
    required_fields = parameters_schema.get("required", [])

    for field_name in required_fields:
        if field_name not in arguments:
            return {
                "is_valid": False,
                "reason": f"Missing required argument: {field_name}",
            }

    for argument_name, argument_value in arguments.items():
        if argument_name not in properties:
            return {
                "is_valid": False,
                "reason": f"Unexpected argument: {argument_name}",
            }

        expected_type = properties[argument_name].get("type")

        if expected_type == "number" and not isinstance(argument_value, (int, float)):
            return {
                "is_valid": False,
                "reason": f"Argument '{argument_name}' must be numeric.",
            }

        if expected_type == "string" and not isinstance(argument_value, str):
            return {
                "is_valid": False,
                "reason": f"Argument '{argument_name}' must be a string.",
            }

    return {
        "is_valid": True,
        "reason": "Valid tool call.",
    }


# ---------- Accuracy Evaluation ----------
def _numbers_match(left_value, right_value) -> bool:
    if isinstance(left_value, (int, float)) and isinstance(right_value, (int, float)):
        return math.isclose(left_value, right_value, rel_tol=0.0, abs_tol=FLOAT_TOLERANCE)

    return left_value == right_value


def compare_tool_calls(predicted_call: dict | None, expected_call: dict) -> dict:
    if predicted_call is None:
        return {
            "is_correct": False,
            "reason": "No predicted tool call to compare.",
        }

    if predicted_call.get("name") != expected_call.get("name"):
        return {
            "is_correct": False,
            "reason": (
                f"Wrong tool name. Expected '{expected_call.get('name')}', "
                f"got '{predicted_call.get('name')}'."
            ),
        }

    predicted_arguments = predicted_call.get("arguments", {})
    expected_arguments = expected_call.get("arguments", {})

    for argument_name, expected_value in expected_arguments.items():
        if argument_name not in predicted_arguments:
            return {
                "is_correct": False,
                "reason": f"Missing expected argument: {argument_name}",
            }

        predicted_value = predicted_arguments[argument_name]

        if not _numbers_match(predicted_value, expected_value):
            return {
                "is_correct": False,
                "reason": (
                    f"Wrong value for '{argument_name}'. "
                    f"Expected {expected_value}, got {predicted_value}."
                ),
            }

    return {
        "is_correct": True,
        "reason": "Predicted call matches expected call.",
    }


# ---------- Full Evaluation ----------
def evaluate_response(output_text: str, expected_call: dict, tool_declarations: list) -> dict:
    parsed_call = parse_tool_call(output_text)
    validation_result = validate_tool_call(parsed_call, tool_declarations)

    if not validation_result["is_valid"]:
        return {
            "raw_output": output_text,
            "parsed_call": parsed_call,
            "adherence_pass": False,
            "adherence_reason": validation_result["reason"],
            "accuracy_pass": False,
            "accuracy_reason": "Cannot score accuracy because adherence failed.",
        }

    accuracy_result = compare_tool_calls(parsed_call, expected_call)

    return {
        "raw_output": output_text,
        "parsed_call": parsed_call,
        "adherence_pass": True,
        "adherence_reason": validation_result["reason"],
        "accuracy_pass": accuracy_result["is_correct"],
        "accuracy_reason": accuracy_result["reason"],
    }