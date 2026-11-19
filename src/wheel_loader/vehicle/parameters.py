"""SI parameters; loading a file never imports a simulator or a reference vehicle."""

import math
from pathlib import Path

import yaml


def resolve_values(value):
    """Resolve annotated SI values at the configuration boundary; never infer sources."""
    if isinstance(value, dict):
        if "value" in value:
            if not {"unit", "source", "confidence", "notes"} <= set(value):
                raise ValueError("Physical parameter lacks units/source/confidence")
            if value["source"] not in {
                "MANUFACTURER",
                "SERVICE_MANUAL",
                "BROCHURE",
                "MEASURED",
                "DERIVED",
                "ESTIMATED",
                "CALIBRATED",
            }:
                raise ValueError("Unknown physical parameter source")
            return value["value"]
        return {k: resolve_values(v) for k, v in value.items()}
    if isinstance(value, list):
        return [resolve_values(v) for v in value]
    return value


def load_parameters(path: str | Path) -> dict:
    parameters = resolve_values(yaml.safe_load(Path(path).read_text()))
    required = {
        "vehicle",
        "articulation",
        "boom",
        "linkage",
        "bucket",
        "geometry",
        "masses",
        "contact",
        "wheels",
        "solver",
    }
    if (
        not isinstance(parameters, dict)
        or not required <= set(parameters)
        or set(parameters)
        - required
        - {
            "model_version",
            "bodies",
            "cylinders",
            "maneuver",
            "actuators",
            "collision_geometry",
            "cylinder_bodies",
        }
    ):
        raise ValueError("Vehicle parameter sections differ from the WA320 schema")

    def finite(value):
        if isinstance(value, dict):
            for item in value.values():
                finite(item)
        elif isinstance(value, list):
            for item in value:
                finite(item)
        elif (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ValueError("Every vehicle parameter must be finite numeric SI data")

    for section in required:
        finite(parameters[section])
    if "cylinders" in parameters:
        finite(parameters["cylinders"])
    v, m = parameters["vehicle"], parameters["masses"]
    if any(value <= 0 for value in v.values()) or any(
        value <= 0 for value in m.values()
    ):
        raise ValueError("Vehicle dimensions and masses must be positive")
    total = sum(m.values()) + 3 * m["wheel"]
    if not math.isclose(total, v["mass"], abs_tol=1e-6):
        raise ValueError("Mass allocation must equal operating mass")
    for name in ("articulation", "boom", "linkage", "bucket"):
        limits = parameters[name]
        if (
            limits["min_angle"] >= limits["max_angle"]
            or not limits["min_angle"] <= 0 <= limits["max_angle"]
        ):
            raise ValueError(f"{name}: invalid or non-neutral angle limits")
    for section, keys in {
        "boom": ["length", "rail_spacing", "rail_thickness"],
        "linkage": [
            "bellcrank_upper_length",
            "bellcrank_lower_length",
            "rod_length",
            "bar_thickness",
        ],
        "bucket": ["plate_thickness"],
        "wheels": ["max_velocity", "max_torque"],
        "solver": [
            "dt",
            "position_iterations",
            "velocity_iterations",
            "wheel_damping",
            "joint_stiffness",
            "joint_damping",
        ],
    }.items():
        if any(parameters[section][key] <= 0 for key in keys):
            raise ValueError(
                f"{section}: physical lengths, rates and solver values must be positive"
            )
    if not 0 < parameters["linkage"]["bellcrank_pivot_fraction"] < 1:
        raise ValueError("Bellcrank pivot must lie along the boom")
    c = parameters["contact"]
    if (
        not 0 <= c["dynamic_friction"] <= c["static_friction"]
        or not 0 <= c["restitution"] <= 1
    ):
        raise ValueError("Invalid rigid contact coefficients")
    sides = parameters["wheels"]["collision_sides"]
    if not isinstance(sides, int) or not 16 <= sides <= 128:
        raise ValueError("Tire hull resolution must be an integer from 16 to 128")
    if "bodies" in parameters:
        from .mass_properties import body_properties

        body_properties(parameters)  # Reject invalid allocations/inertia before export.
        if "cylinder_bodies" in parameters:
            import numpy as np

            from .mass_properties import body_transforms

            frames = body_transforms(parameters, 0, 0, 0)
            for name, spec in parameters["cylinder_bodies"].items():
                base, r = frames[spec["parent"]]
                tip, rt = frames[spec["tip_body"]]
                length = np.linalg.norm(
                    tip + rt @ spec["tip"] - base - r @ spec["base"]
                )
                if not np.isclose(length, spec["reference_length"], atol=1e-9, rtol=0):
                    raise ValueError(
                        f"{name}: neutral cylinder pin length must close before import"
                    )

    return parameters
