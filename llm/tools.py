def get_tool_declarations():
    return [
        {
            "type": "function",
            "function": {
                "name": "move_spot",
                "description": "Moves Spot forward or backward by a specified amount of meters.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "meters": {
                            "type": "number",
                            "description": "Specified number of meters to move. If meters is positive, Spot moves forward. If meters is negative, Spot moves backward.",
                        }
                    },
                    "required": ["meters"],
                }
            }
        },
                {
            "type": "function",
            "function": {
                "name": "rotate_spot",
                "description": "Rotates Spot by a specified amount of degrees.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "degrees": {
                            "type": "number",
                            "description": "Specified distance of degrees to rotate. If degrees is positive, Spot rotates clockwise. If degrees is negative, Spot rotates counter-clockwise.",
                        }
                    },
                    "required": ["degrees"],
                }
            }
        },
    ]