"""Conservative rear-body clearance for provisional visual cylinder geometry."""

from pathlib import Path

import numpy as np

from wheel_loader.vehicle.parameters import load_parameters


def test_visual_lift_envelope_clears_rear_body():
    p = load_parameters(
        Path(__file__).resolve().parents[2] / "configs/vehicle/wa320.yaml"
    )
    # A half-space certificate: all pin centres stay ahead of rear box's front
    # face, even after expanding the entire segment by the assembly's bound.
    front_face = (
        p["geometry"]["rear_box_center"][0] + p["geometry"]["rear_box_size"][0] / 2
    )
    assembly_radius_bound = 0.13  # includes barrel/gland/eye/pin display geometry
    minimum = float("inf")
    for a in np.linspace(
        p["articulation"]["min_angle"], p["articulation"]["max_angle"], 121
    ):
        yaw = np.array(
            [[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]
        )
        for q in np.linspace(
            p["boom"]["initial_elevation"],
            p["boom"]["initial_elevation"] + p["boom"]["max_angle"],
            81,
        ):
            lift = np.array(
                [[np.cos(q), 0, -np.sin(q)], [0, 1, 0], [np.sin(q), 0, np.cos(q)]]
            )
            for sign in (-1, 1):
                base = np.array(p["geometry"]["lift_cylinder_base"])
                rod = np.array(p["geometry"]["lift_cylinder_rod_pin"])
                base[1] *= sign
                rod[1] *= sign
                pivot = np.array([p["boom"]["pivot"][0], 0, p["boom"]["pivot"][1]])
                ends = [yaw @ base, yaw @ (pivot + lift @ rod)]
                minimum = min(
                    minimum,
                    *(end[0] - front_face - assembly_radius_bound for end in ends),
                )
    assert minimum > 0.05  # Approximate display clearance, not a machine tolerance.
