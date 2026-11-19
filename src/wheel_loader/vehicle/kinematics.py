"""Original planar Z-bar geometry. Positive X/Z angles raise a forward-pointing bar.

URDF rotation axes are -Y for elevation/curl and +Z for chassis articulation.
The bucket and rod are passive outputs, not independently commanded joints.
"""

import math
from dataclasses import dataclass

import numpy as np


def rotate(angle: float, vector) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]]) @ np.asarray(vector, dtype=float)


def circle_intersection(center0, radius0, center1, radius1, branch=-1):
    """Return a fixed assembly branch; reject unreachable and toggle configurations."""
    a, b = np.asarray(center0, dtype=float), np.asarray(center1, dtype=float)
    delta = b - a
    distance = float(np.linalg.norm(delta))
    if (
        min(radius0, radius1) <= 0
        or not abs(radius0 - radius1) + 1e-8 < distance < radius0 + radius1 - 1e-8
    ):
        raise ValueError("Unreachable or singular Z-bar closure")
    axis = delta / distance
    along = (radius0**2 - radius1**2 + distance**2) / (2 * distance)
    height = math.sqrt(max(0.0, radius0**2 - along**2))
    return a + along * axis + branch * height * np.array([-axis[1], axis[0]])


@dataclass(frozen=True)
class LinkagePose:
    boom_elevation: float
    bellcrank_angle: float
    bucket_angle: float  # relative to boom
    rod_angle: float  # relative to boom
    cylinder_length: float  # geometric length only; no hydraulic force model
    boom_pin: np.ndarray
    bellcrank_pin: np.ndarray
    upper_pin: np.ndarray
    lower_pin: np.ndarray
    bucket_pin: np.ndarray
    bucket_anchor: np.ndarray
    lip: np.ndarray


def solve_linkage(parameters: dict, boom_angle=0.0, tilt_angle=0.0) -> LinkagePose:
    """Commands are displacements from nominal boom and bellcrank angles [rad].

    Geometry is returned in front-frame X/Z, relative to the articulation root.
    """
    if not all(math.isfinite(value) for value in (boom_angle, tilt_angle)):
        raise ValueError("Nonfinite linkage command")
    for section, value in (("boom", boom_angle), ("linkage", tilt_angle)):
        if (
            not parameters[section]["min_angle"]
            <= value
            <= parameters[section]["max_angle"]
        ):
            raise ValueError(f"{section} command exceeds configured limits")
    b, k = parameters["boom"], parameters["linkage"]
    phi = k["initial_bellcrank_angle"] + tilt_angle
    c = np.array(
        k.get("bellcrank_pivot", [b["length"] * k["bellcrank_pivot_fraction"], 0.0])
    )
    p = np.array([b["length"], 0.0])
    upper = c + rotate(phi, [k["bellcrank_upper_length"], 0.0])
    lower = c + rotate(phi, k.get("rod_pin_local", [-k["bellcrank_lower_length"], 0.0]))
    attachment = np.asarray(k["bucket_anchor"])
    f = circle_intersection(
        p, float(np.linalg.norm(attachment)), lower, k["rod_length"]
    )
    beta = math.atan2(*(f - p)[::-1]) - math.atan2(attachment[1], attachment[0])
    alpha = math.atan2(*(f - lower)[::-1])
    q = b["initial_elevation"] + boom_angle
    origin = np.asarray(b["pivot"])
    to_front = lambda point: origin + rotate(q, point)
    cylinder = np.linalg.norm(rotate(q, upper) - np.asarray(k["tilt_cylinder_base"]))
    return LinkagePose(
        q,
        phi,
        beta,
        alpha,
        float(cylinder),
        origin,
        to_front(c),
        to_front(upper),
        to_front(lower),
        to_front(p),
        to_front(f),
        to_front(p + rotate(beta, parameters["bucket"]["lip"])),
    )


def passive_joint_positions(
    parameters, boom_angle=0.0, tilt_angle=0.0, articulation=0.0
) -> dict[str, float]:
    pose, neutral = (
        solve_linkage(parameters, boom_angle, tilt_angle),
        solve_linkage(parameters),
    )
    beta = pose.bucket_angle - neutral.bucket_angle
    if (
        not parameters["bucket"]["min_angle"]
        <= beta
        <= parameters["bucket"]["max_angle"]
    ):
        raise ValueError("Linkage violates passive bucket joint envelope")
    result = {
        "boom_joint": boom_angle,
        "tilt_joint": tilt_angle,
        "bucket_joint": beta,
        "rod_joint": (pose.rod_angle - pose.bellcrank_angle)
        - (neutral.rod_angle - neutral.bellcrank_angle),
    }

    if "cylinder_bodies" in parameters:
        from .mass_properties import body_transforms

        frames = body_transforms(parameters, boom_angle, tilt_angle, articulation)
        for name, spec in parameters["cylinder_bodies"].items():
            b, r = frames[spec["parent"]]
            tip, rt = frames[spec["tip_body"]]
            d = r.T @ (tip + rt @ spec["tip"] - b) - spec["base"]
            current = (
                math.atan2(d[0], d[2])
                if spec["axis"] == "Y"
                else math.atan2(d[1], d[0])
            )
            # URDF zero hinge orientation is the nominal empty attachment pose.
            neutral_frames = body_transforms(parameters, 0, 0, 0)
            b0, r0 = neutral_frames[spec["parent"]]
            t0, rt0 = neutral_frames[spec["tip_body"]]
            d0 = r0.T @ (t0 + rt0 @ spec["tip"] - b0) - spec["base"]
            initial = (
                math.atan2(d0[0], d0[2])
                if spec["axis"] == "Y"
                else math.atan2(d0[1], d0[0])
            )
            result[name + "_hinge_joint"] = math.atan2(
                math.sin(current - initial), math.cos(current - initial)
            )
            result[name + "_slider_joint"] = float(
                np.linalg.norm(d) - spec["reference_length"]
            )
    return result
