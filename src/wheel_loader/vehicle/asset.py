"""Generate original primitive geometry; no Komodo, third-party CAD or mesh inputs.

URDF is a tree plus an explicit closure sidecar. Import integration must add
that closure; the bare URDF must never be called a complete Z-bar dynamics model.
"""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from .kinematics import solve_linkage


def vector(value):
    return " ".join(f"{float(v):.12g}" for v in value)


def build_asset(p: dict, output: Path):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    root = ET.Element("robot", name="wa320")
    root.append(
        ET.Comment(
            "Original estimated geometry. Tree requires closure.json in supported simulation integration."
        )
    )
    colors = {
        "yellow": "0.95 0.61 0.04 1",
        "dark": "0.12 0.13 0.14 1",
        "glass": "0.16 0.32 0.4 1",
        "steel": "0.48 0.5 0.52 1",
    }
    for name, color in colors.items():
        mat = ET.SubElement(root, "material", name=name)
        ET.SubElement(mat, "color", rgba=color)

    def inertia(link, mass, size, center=(0, 0, 0), cylinder=False):
        node = ET.SubElement(link, "inertial")
        ET.SubElement(node, "origin", xyz=vector(center))
        ET.SubElement(node, "mass", value=str(mass))
        if cylinder:
            radius, width = size
            ixx = izz = mass * (3 * radius**2 + width**2) / 12
            iyy = mass * radius**2 / 2
        else:
            x, y, z = size
            ixx = mass * (y * y + z * z) / 12
            iyy = mass * (x * x + z * z) / 12
            izz = mass * (x * x + y * y) / 12
        ET.SubElement(
            node,
            "inertia",
            ixx=str(ixx),
            iyy=str(iyy),
            izz=str(izz),
            ixy="0",
            ixz="0",
            iyz="0",
        )

    def shape(
        link,
        size,
        xyz=(0, 0, 0),
        rpy=(0, 0, 0),
        material="yellow",
        cylinder=False,
        collision=True,
    ):
        for kind in ("visual", "collision") if collision else ("visual",):
            node = ET.SubElement(link, kind)
            ET.SubElement(node, "origin", xyz=vector(xyz), rpy=vector(rpy))
            geom = ET.SubElement(node, "geometry")
            if cylinder:
                ET.SubElement(
                    geom, "cylinder", radius=str(size[0]), length=str(size[1])
                )
            else:
                ET.SubElement(geom, "box", size=vector(size))
            if kind == "visual":
                ET.SubElement(node, "material", name=material)

    def joint(
        name,
        parent,
        child,
        xyz,
        axis,
        lower,
        upper,
        effort,
        velocity,
        angle=0.0,
        continuous=False,
    ):
        node = ET.SubElement(
            root, "joint", name=name, type="continuous" if continuous else "revolute"
        )
        ET.SubElement(node, "parent", link=parent)
        ET.SubElement(node, "child", link=child)
        ET.SubElement(node, "origin", xyz=vector(xyz), rpy=vector([0, -angle, 0]))
        ET.SubElement(node, "axis", xyz=vector(axis))
        attrs = {"effort": str(effort), "velocity": str(velocity)}
        if not continuous:
            attrs.update(lower=str(lower), upper=str(upper))
        ET.SubElement(node, "limit", **attrs)

    v, g, m = p["vehicle"], p["geometry"], p["masses"]
    rear = ET.SubElement(root, "link", name="rear_frame")
    inertia(rear, m["rear_frame"], g["rear_box_size"], g["rear_box_center"])
    shape(rear, g["rear_box_size"], g["rear_box_center"])
    shape(rear, g["cab_size"], g["cab_center"], material="glass")
    front = ET.SubElement(root, "link", name="front_frame")
    inertia(front, m["front_frame"], g["front_box_size"], g["front_box_center"])
    for side in (-1, 1):
        shape(
            front,
            [g["front_box_size"][0], g["front_rail_width"], g["front_box_size"][2]],
            [
                g["front_box_center"][0],
                side * g["front_rail_spacing"] / 2,
                g["front_box_center"][2],
            ],
        )
    shape(
        front,
        g["front_axle_box_size"],
        [v["wheelbase"] / 2, 0, v["wheel_radius"] - v["articulation_height"]],
    )
    a = p["articulation"]
    joint(
        "articulation_joint",
        "rear_frame",
        "front_frame",
        [0, 0, 0],
        [0, 0, 1],
        a["min_angle"],
        a["max_angle"],
        a["max_torque"],
        a["max_velocity"],
    )
    for axle, sign in [("rear", -1), ("front", 1)]:
        for side, left in [("left", 1), ("right", -1)]:
            name = f"{axle}_{side}_wheel"
            wheel = ET.SubElement(root, "link", name=name)
            inertia(
                wheel, m["wheel"], [v["wheel_radius"], v["wheel_width"]], cylinder=True
            )
            shape(
                wheel,
                [v["wheel_radius"], v["wheel_width"]],
                rpy=[math.pi / 2, 0, 0],
                cylinder=True,
                material="dark",
            )
            joint(
                name + "_joint",
                axle + "_frame",
                name,
                [
                    sign * v["wheelbase"] / 2,
                    left * v["track_width"] / 2,
                    v["wheel_radius"] - v["articulation_height"],
                ],
                [0, 1, 0],
                0,
                0,
                p["wheels"]["max_torque"],
                p["wheels"]["max_velocity"],
                continuous=True,
            )
    b, k = p["boom"], p["linkage"]
    pose = solve_linkage(p)
    boom = ET.SubElement(root, "link", name="boom")
    inertia(
        boom,
        m["boom"],
        [b["length"], b["rail_spacing"] + b["rail_thickness"], b["rail_thickness"]],
        [b["length"] / 2, 0, 0],
    )
    for side in (-1, 1):
        shape(
            boom,
            [b["length"], b["rail_thickness"], b["rail_thickness"]],
            [b["length"] / 2, side * b["rail_spacing"] / 2, 0],
        )
    joint(
        "boom_joint",
        "front_frame",
        "boom",
        [b["pivot"][0], 0, b["pivot"][1]],
        [0, -1, 0],
        b["min_angle"],
        b["max_angle"],
        b["max_torque"],
        b["max_velocity"],
        b["initial_elevation"],
    )
    crank = ET.SubElement(root, "link", name="bellcrank")
    thickness = k["bar_thickness"]
    crank_length = k["bellcrank_upper_length"] + k["bellcrank_lower_length"]
    center = (k["bellcrank_upper_length"] - k["bellcrank_lower_length"]) / 2
    inertia(crank, m["bellcrank"], [crank_length, thickness, thickness], [center, 0, 0])
    shape(crank, [crank_length, thickness, thickness], [center, 0, 0], material="dark")
    joint(
        "tilt_joint",
        "boom",
        "bellcrank",
        [b["length"] * k["bellcrank_pivot_fraction"], 0, 0],
        [0, -1, 0],
        k["min_angle"],
        k["max_angle"],
        k["max_torque"],
        k["max_velocity"],
        k["initial_bellcrank_angle"],
    )
    rod = ET.SubElement(root, "link", name="rod")
    inertia(
        rod,
        m["rod"],
        [k["rod_length"], thickness, thickness],
        [k["rod_length"] / 2, 0, 0],
    )
    shape(
        rod,
        [k["rod_length"], thickness, thickness],
        [k["rod_length"] / 2, 0, 0],
        material="steel",
    )
    joint(
        "rod_joint",
        "bellcrank",
        "rod",
        [-k["bellcrank_lower_length"], 0, 0],
        [0, -1, 0],
        k["rod_min_angle"],
        k["rod_max_angle"],
        0,
        k["passive_max_velocity"],
        pose.rod_angle - pose.bellcrank_angle,
    )
    bucket = ET.SubElement(root, "link", name="bucket")
    bc = p["bucket"]
    x0, z0 = bc["back_bottom"]
    x1, z1 = bc["lip"]
    _, zt = bc["back_top"]
    shell_size = [x1 - x0, v["bucket_width"], zt - z0]
    shell_center = [(x0 + x1) / 2, 0, (z0 + zt) / 2]
    inertia(bucket, m["bucket"], shell_size, shell_center)
    plate = bc["plate_thickness"]
    length = math.hypot(x1 - x0, z1 - z0)
    floor_angle = math.atan2(z1 - z0, x1 - x0)
    shape(
        bucket,
        [length, v["bucket_width"], plate],
        [(x0 + x1) / 2, 0, (z0 + z1) / 2],
        rpy=[0, -floor_angle, 0],
    )
    shape(bucket, [plate, v["bucket_width"], zt - z0], [x0, 0, (z0 + zt) / 2])
    for side in (-1, 1):
        shape(
            bucket,
            [x1 - x0, plate, zt - z0],
            [(x0 + x1) / 2, side * (v["bucket_width"] - plate) / 2, (z0 + zt) / 2],
        )
    joint(
        "bucket_joint",
        "boom",
        "bucket",
        [b["length"], 0, 0],
        [0, -1, 0],
        bc["min_angle"],
        bc["max_angle"],
        0,
        k["passive_max_velocity"],
        pose.bucket_angle,
    )
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(
        output / "wa320.urdf", encoding="unicode", xml_declaration=True
    )
    (output / "wa320.xacro").write_text("""<?xml version="1.0"?>
<!-- Generate parameters with scripts/build_wa320.py; this wrapper expands the same tree. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="wa320">
  <xacro:include filename="wa320.urdf"/>
</robot>
""")
    closure = {
        "joint": "zbar_closure",
        "type": "revolute",
        "body0": "rod",
        "body1": "bucket",
        "local_pos0": [k["rod_length"], 0, 0],
        "local_pos1": [k["bucket_anchor"][0], 0, k["bucket_anchor"][1]],
        "axis": "Y",
        "exclude_from_articulation": True,
        "warning": "Required loop joint. Bare URDF omits it; passive rod/bucket must NOT be driven.",
    }
    (output / "closure.json").write_text(json.dumps(closure, indent=2) + "\n")
    return output / "wa320.urdf"
