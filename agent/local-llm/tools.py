def get_tool_declarations():
    return [
        # ---------- SPOT TOOLS ----------
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

        # ---------- DRONE TOOLS ----------
        {
            "type": "function",
            "function": {
                "name": "move_forward_drone",
                "description": "Moves Drone forward or backward by a specified amount of meters.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "meters": {
                            "type": "number",
                            "description": "Specified distance of meters to move. If meters is positive, Drone moves forward. If meters is negative, Drone moves backward..",
                        }
                    },
                    "required": ["meters"],
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "move_lateral_drone",
                "description": "Moves Drone left or right by a specified amount of meters.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "meters": {
                            "type": "number",
                            "description": "Specified distance of meters to move. If meters is positive, Drone moves left. If meters is negative, Drone moves right",
                        }
                    },
                    "required": ["meters"],
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "raise_altitude_drone",
                "description": "Raises or lowers the altitude of the Drone by a specified amount of meters.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "meters": {
                            "type": "number",
                            "description": "Specified distance of meters to ascend or descend. If meters is positive, Drone ascends. If meters is negative, Drone descends",
                        }
                    },
                    "required": ["meters"],
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "rotate_drone",
                "description": "Rotates Drone by a specified amount of degrees.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "degrees": {
                            "type": "number",
                            "description": "Specified distance of degrees to rotat. If degrees is positive, Drone rotates counter-clockwise. If degrees is negative, Drone rotates clockwisee",
                        }
                    },
                    "required": ["degrees"],
                }
            }
        },
    ]