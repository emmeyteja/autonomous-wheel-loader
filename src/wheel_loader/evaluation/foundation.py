"""Pure geometry checks. Passing these does not establish physics correctness."""

import numpy as np

from wheel_loader.vehicle.kinematics import passive_joint_positions, solve_linkage


def check_geometry(p):
    maximum_residual = 0.0
    minimum_lip = float("inf")
    for boom in np.linspace(p["boom"]["min_angle"], p["boom"]["max_angle"], 21):
        for tilt in np.linspace(
            p["linkage"]["min_angle"], p["linkage"]["max_angle"], 33
        ):
            pose = solve_linkage(p, float(boom), float(tilt))
            passive_joint_positions(p, float(boom), float(tilt))
            residual = abs(
                np.linalg.norm(pose.bucket_anchor - pose.lower_pin)
                - p["linkage"]["rod_length"]
            )
            maximum_residual = max(maximum_residual, float(residual))
            minimum_lip = min(
                minimum_lip, float(pose.lip[1] + p["vehicle"]["articulation_height"])
            )
    return {
        "vehicle": "wa320",
        "mode": "foundation geometry validation; no training",
        "samples": 21 * 33,
        "max_loop_residual_m": maximum_residual,
        "minimum_lip_height_over_joint_envelope_m": minimum_lip,
        "physics_validated": False,
        "limitations": [
            "estimated pivots and mass distribution",
            "joint envelope is not a collision-free workspace",
            "equivalent rotary drives; no hydraulic/power-sharing model",
            "rigid pile cannot be excavated",
        ],
    }
