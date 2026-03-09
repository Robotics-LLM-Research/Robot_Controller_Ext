def get_tool_declarations():
    return [
        # ---------- SPOT TOOLS ----------
        {
            "name": "move_spot",
            "description": "Moves Spot forward or backward by a specified amount of meters.",
            "parameters" : {
                "type": "object",
                "properties": {
                    "meters": {
                        "type": "number",
                        "description": "Specified number of meters to move. If meters is positive, Spot moves forward. If meters is negative, Spot moves backward.",
                    }
                },
                "required": ["meters"],
            }
        },

        {
            "name": "rotate_spot",
            "description": "Rotates Spot by a specified amount of degrees.",
            "parameters" : {
                "type": "object",
                "properties": {
                    "degrees": {
                        "type": "number",
                        "description": "Specified distance of degrees to rotate. If degrees is positive, Spot rotates clockwise. If degrees is negative, Spot rotates counter-clockwise.",
                    }
                },
                "required": ["degrees"],
            }
        },

        # ---------- DRONE TOOLS ----------
        # --- Motion ---
        {
            "name": "move_forward_drone",
            "description": "Moves Drone forward or backward by a specified amount of meters.",
            "parameters" : {
                "type": "object",
                "properties": {
                    "meters": {
                        "type": "number",
                        "description": "Specified distance of meters to move. If meters is positive, Drone moves forward. If meters is negative, Drone moves backward.",
                    }
                },
                "required": ["meters"],
            }
        }, 
        
        {
            "name": "move_lateral_drone",
            "description": "Moves Drone left or right by a specified amount of meters.",
            "parameters" : {
                "type": "object",
                "properties": {
                    "meters": {
                        "type": "number",
                        "description": "Specified distance of meters to move. If meters is positive, Drone moves left. If meters is negative, Drone moves right",
                    }
                },
                "required": ["meters"],
            },
        },

        {
            "name": "raise_altitude_drone",
            "description": "Raises or lowers the altitude of the Drone by a specified amount of meters.",
            "parameters" : {
                "type": "object",
                "properties": {
                    "meters": {
                        "type": "number",
                        "description": "Specified distance of meters to ascend or descend. If meters is positive, Drone ascends. If meters is negative, Drone descends",
                    }
                },
                "required": ["meters"],
            },
        },

        {
            "name": "rotate_drone",
            "description": "Rotates Drone by a specified amount of degrees.",
            "parameters" : {
                "type": "object",
                "properties": {
                    "degrees": {
                        "type": "number",
                        "description": "Specified distance of degrees to rotat. If degrees is positive, Drone rotates counter-clockwise. If degrees is negative, Drone rotates clockwisee",
                    }
                },
                "required": ["degrees"],
            },
        },

        # --- Sensors ---
        {
            "name": "look_drone",
            "description": "Moves the Drone's on-board camera based on polar coordinates. At (0,0) the camera is in its neutral position pointing center.",
            "parameters" : {
                "type": "object",
                "properties": {
                    "x": {
                        "type": "number",
                        "description": "Limits of [-1, 1]. If x is negative it points left, if x is positive it points right.",
                    },
                    "y": {
                        "type" : "number",
                        "description" : "Limits of [-1, 1]. If x is negative it points left, if x is positive it points right."
                    }
                },
                "required": ["x", "y"],
            },
        }
    ]