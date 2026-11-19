"""Deterministic kinematic signature; this does not identify true machine geometry."""

import numpy as np

from wheel_loader.vehicle.kinematics import passive_joint_positions, solve_linkage
from wheel_loader.vehicle.mass_properties import body_transforms


def linkage_signature(p, count=51):
    if count < 2:
        raise ValueError("At least two signature poses required")
    rows = []
    for f in np.linspace(0, 1, count):
        boom = (1 - f) * p["maneuver"]["lower_attack"][0] + f * p["maneuver"][
            "raise_dump"
        ][0]
        tilt = (1 - f) * p["maneuver"]["lower_attack"][1] + f * p["maneuver"][
            "raise_dump"
        ][1]
        passive_joint_positions(p, boom, tilt)
        pose = solve_linkage(p, boom, tilt)
        frames = body_transforms(p, boom, tilt)
        gaps = []
        lengths = {}
        for name, spec in p["cylinder_bodies"].items():
            b, r = frames[spec["parent"]]
            tip, rt = frames[spec["tip_body"]]
            base = b + r @ spec["base"]
            target = tip + rt @ spec["tip"]
            rod, _ = frames[name + "_rod"]
            gaps.append(float(np.linalg.norm(rod - target)))
            length = float(np.linalg.norm(target - base))
            lengths[name + "_length_m"] = length
            c = p["cylinders"][spec["kind"]]
            lengths[name + "_extension_m"] = length - c["pin_distance_min"]
            if (
                not c["pin_distance_min"] - 1e-8
                <= length
                <= c["pin_distance_max"] + 1e-8
            ):
                raise ValueError(f"{name}: signature exceeds cylinder stroke")
        residual = abs(
            np.linalg.norm(pose.bucket_anchor - pose.lower_pin)
            - p["linkage"]["rod_length"]
        )
        rows.append(
            dict(
                boom_rad=float(boom),
                bellcrank_rad=float(tilt),
                bucket_rad=float(pose.bucket_angle),
                tip_x_m=float(pose.lip[0]),
                tip_z_m=float(pose.lip[1] + p["vehicle"]["articulation_height"]),
                zbar_gap_m=float(residual),
                maximum_cylinder_pin_gap_m=max(gaps),
                **lengths,
            )
        )
    return rows
