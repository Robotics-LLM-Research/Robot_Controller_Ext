import random



# ---------- Prompt Builders ----------
def _build_spot_move_prompt(meters: float) -> str:
    direction_word = "forward" if meters > 0 else "backward"
    magnitude = abs(meters)

    prompt_templates = [
        f"Move Spot {magnitude} meters {direction_word}.",
        f"Tell Spot to move {direction_word} by {magnitude} meters.",
        f"Make Spot go {direction_word} {magnitude} meters.",
    ]

    return random.choice(prompt_templates)


def _build_spot_rotate_prompt(degrees: float) -> str:
    direction_word = "clockwise" if degrees > 0 else "counter-clockwise"
    magnitude = abs(degrees)

    prompt_templates = [
        f"Rotate Spot {magnitude} degrees {direction_word}.",
        f"Tell Spot to turn {direction_word} by {magnitude} degrees.",
        f"Make Spot rotate {direction_word} {magnitude} degrees.",
    ]

    return random.choice(prompt_templates)


# ---------- Case Generators ----------
def generate_spot_move_cases(values: list[float]) -> list:
    test_cases = []

    for meters in values:
        test_cases.append(
            {
                "prompt": _build_spot_move_prompt(meters),
                "expected_call": {
                    "name": "move_spot",
                    "arguments": {"meters": meters},
                },
                "category": "spot_move",
            }
        )

    return test_cases


def generate_spot_rotate_cases(values: list[float]) -> list:
    test_cases = []

    for degrees in values:
        test_cases.append(
            {
                "prompt": _build_spot_rotate_prompt(degrees),
                "expected_call": {
                    "name": "rotate_spot",
                    "arguments": {"degrees": degrees},
                },
                "category": "spot_rotate",
            }
        )

    return test_cases


def generate_test_cases(move_values, rotate_values) -> list:
    test_cases = []
    test_cases.extend(generate_spot_move_cases(move_values))
    test_cases.extend(generate_spot_rotate_cases(rotate_values))

    random.shuffle(test_cases)
    return test_cases