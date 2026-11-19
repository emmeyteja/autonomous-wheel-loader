"""Primitive COM/inertia and planar support loads; no simulator dependency.

All tensors are about each body's COM, expressed in that body's axes.
Compound inertia uses rotated primitive tensors and the parallel-axis theorem.
"""

import math

import numpy as np

from .kinematics import solve_linkage


def rotation3(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return (
        np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    )


def compound_properties(mass, primitives):
    fractions = np.array([p["fraction"] for p in primitives], float)
    if (
        mass <= 0
        or not np.isfinite(mass)
        or not np.isfinite(fractions).all()
        or np.any(fractions <= 0)
        or not np.isclose(fractions.sum(), 1, atol=1e-10)
    ):
        raise ValueError("Primitive mass fractions must be positive and sum to one")
    centers = np.array([p["center"] for p in primitives], float)
    if centers.shape != (len(primitives), 3) or not np.isfinite(centers).all():
        raise ValueError("Invalid primitive COM")
    com = fractions @ centers
    tensor = np.zeros((3, 3))
    for f, c, p in zip(fractions, centers, primitives):
        m = mass * f
        size = np.array(p["size"], float)
        if not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError("Invalid primitive dimensions")
        if p["shape"] == "box" and size.shape == (3,):
            x, y, z = size
            diagonal = m / 12 * np.array([y * y + z * z, x * x + z * z, x * x + y * y])
        elif p["shape"] == "cylinder" and size.shape == (2,):
            radius, length = size  # Cylinder local Z axis.
            diagonal = m * np.array(
                [(3 * radius**2 + length**2) / 12] * 2 + [radius**2 / 2]
            )
        elif p["shape"] == "annulus" and size.shape == (3,):
            outer, inner, length = size
            if inner >= outer:
                raise ValueError("Annulus bore must be smaller than outer radius")
            diagonal = m * np.array(
                [(3 * (outer**2 + inner**2) + length**2) / 12] * 2
                + [(outer**2 + inner**2) / 2]
            )
        else:
            raise ValueError("Unsupported inertia primitive")
        r = rotation3(p["rpy"])
        delta = c - com
        tensor += r @ np.diag(diagonal) @ r.T + m * (
            (delta @ delta) * np.eye(3) - np.outer(delta, delta)
        )
    eigen = np.linalg.eigvalsh(tensor)
    if (
        not np.isfinite(tensor).all()
        or eigen.min() <= 0
        or 2 * eigen.max() > eigen.sum() + 1e-8
    ):
        raise ValueError("Unphysical inertia tensor")
    return {
        "mass_kg": float(mass),
        "com_m": com.tolist(),
        "inertia_kg_m2": tensor.tolist(),
    }


def body_properties(p):
    result, allocation = {}, dict.fromkeys(p["masses"], 0.0)
    for name, b in p["bodies"].items():
        key, fraction = b["mass_key"], b["mass_fraction"]
        if fraction <= 0 or not np.isfinite(fraction):
            raise ValueError("Invalid body mass fraction")
        allocation[key] += fraction
        result[name] = compound_properties(p["masses"][key] * fraction, b["primitives"])
    for key, fraction in allocation.items():
        if not np.isclose(fraction, 4 if key == "wheel" else 1, atol=1e-10):
            raise ValueError(f"Mass group {key} is omitted or double counted")
    if not np.isclose(
        sum(v["mass_kg"] for v in result.values()), p["vehicle"]["mass"], atol=1e-6
    ):
        raise ValueError("Rigid-body mass budget does not close")
    return result


def body_transforms(p, boom=0.15, tilt=-0.05, articulation=0.0):
    """Root is hinge, Z is above ground; XY/XZ transforms match generated URDF."""
    pose = solve_linkage(p, boom, tilt)
    frames = {
        "world": (np.array([0.0, 0.0, p["vehicle"]["articulation_height"]]), np.eye(3))
    }
    for name, b in p["bodies"].items():
        parent = b["parent"]
        if parent not in frames:
            raise ValueError("Body graph is cyclic or not parent ordered")
        offset, r = np.asarray(b["origin"], float), np.eye(3)
        if name == "front_frame":
            r = rotation3([0, 0, articulation])
        elif name == "boom":
            offset = np.array([*p["boom"]["pivot"][:1], 0, p["boom"]["pivot"][1]])
            r = rotation3([0, -pose.boom_elevation, 0])
        elif name == "bellcrank":
            c = p["linkage"].get(
                "bellcrank_pivot",
                [p["boom"]["length"] * p["linkage"]["bellcrank_pivot_fraction"], 0],
            )
            offset = np.array([c[0], 0, c[1]])
            r = rotation3([0, -pose.bellcrank_angle, 0])
        elif name == "rod":
            d = p["linkage"].get(
                "rod_pin_local", [-p["linkage"]["bellcrank_lower_length"], 0]
            )
            offset = np.array([d[0], 0, d[1]])
            r = rotation3([0, -pose.rod_angle + pose.bellcrank_angle, 0])
        elif name == "bucket":
            offset = np.array([p["boom"]["length"], 0, 0])
            r = rotation3([0, -pose.bucket_angle, 0])
        xyz, rp = frames[parent]
        for cylinder, spec in p.get("cylinder_bodies", {}).items():
            if name == cylinder + "_barrel":
                tip_xyz, tip_r = frames[spec["tip_body"]]
                direction = rp.T @ (tip_xyz + tip_r @ spec["tip"] - xyz) - np.asarray(
                    spec["base"]
                )
                angle = (
                    math.atan2(direction[0], direction[2])
                    if spec["axis"] == "Y"
                    else math.atan2(direction[1], direction[0])
                )
                r = rotation3([0, angle, 0] if spec["axis"] == "Y" else [0, 0, angle])
            elif name == cylinder + "_rod":
                tip_xyz, tip_r = frames[spec["tip_body"]]
                length = np.linalg.norm(tip_xyz + tip_r @ spec["tip"] - xyz)
                offset = np.array(
                    [0, 0, length] if spec["axis"] == "Y" else [length, 0, 0]
                )
        frames[name] = (xyz + rp @ offset, rp @ r)
    return frames


def mass_report(p, boom=0.15, tilt=-0.05, payload_kg=0.0, payload_local_m=None):
    """Static two-axle equilibrium, empty or explicit point payload, no contact solver."""
    if payload_kg < 0 or not np.isfinite(payload_kg):
        raise ValueError("Payload must be finite and nonnegative")
    props = body_properties(p)
    transforms = body_transforms(p, boom, tilt)
    for name, entry in props.items():
        xyz, r = transforms[name]
        entry["com_world_m"] = (xyz + r @ entry["com_m"]).tolist()
    total = sum(b["mass_kg"] for b in props.values())
    moment = sum(
        (b["mass_kg"] * np.array(b["com_world_m"]) for b in props.values()),
        start=np.zeros(3),
    )
    rear_names = []
    for name in props:
        node = name
        while node not in ("rear_frame", "front_frame"):
            node = p["bodies"][node]["parent"]
        if node == "rear_frame":
            rear_names.append(name)
    rear_mass = sum(props[n]["mass_kg"] for n in rear_names)
    rear_com = (
        sum(
            (
                props[n]["mass_kg"] * np.array(props[n]["com_world_m"])
                for n in rear_names
            ),
            start=np.zeros(3),
        )
        / rear_mass
    )
    if payload_kg:
        if payload_local_m is None:
            raise ValueError("Explicit payload COM required")
        xyz, r = transforms["bucket"]
        moment += payload_kg * (xyz + r @ payload_local_m)
        total += payload_kg
    com = moment / total
    half = p["vehicle"]["wheelbase"] / 2
    front = total * (com[0] + half) / (2 * half)
    rear = total - front
    return {
        "model": p.get("model_version", "estimated_v0"),
        "body_properties": props,
        "vehicle_mass_kg": total,
        "total_vehicle_COM_m": com.tolist(),
        "front_frame_COM_m": props["front_frame"]["com_world_m"],
        "rear_frame_COM_m": props["rear_frame"]["com_world_m"],
        "rear_body_mass_kg": rear_mass,
        "rear_body_COM_m": rear_com.tolist(),
        "front_axle_load_kg": float(front),
        "rear_axle_load_kg": float(rear),
        "front_axle_load_N": float(front * 9.81),
        "rear_axle_load_N": float(rear * 9.81),
        "front_percentage": float(100 * front / total),
        "rear_percentage": float(100 * rear / total),
        "static_support_positive": bool(min(front, rear) > 0),
        "manufacturer_axle_loads": False,
        "inertias_estimated": True,
    }


def body_path_suffix(p, name):
    """Unambiguous exported body path, including parents (several rods now exist)."""
    if "bodies" not in p:
        return "_" + name
    chain = []
    while name != "world":
        if name in chain:
            raise ValueError("Cyclic body graph")
        chain.append(name)
        name = p["bodies"][name]["parent"]
    return "_" + "_".join(reversed(chain))
