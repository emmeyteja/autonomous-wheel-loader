"""Generate original parametric geometry; no Komodo, third-party CAD or mesh inputs.

URDF is a tree plus an explicit closure sidecar. Import integration must add
that closure; the bare URDF must never be called a complete Z-bar dynamics model.
"""

import itertools
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from .kinematics import solve_linkage
from .tire_visual import tire_meshes, write_tire_meshes


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
    write_tire_meshes(v["wheel_radius"], v["wheel_width"], output / "meshes")
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
            # Retain the cylinder collision and inertia; replace only its visual.
            wheel.remove(wheel.find("visual"))
            for part, (_, _, rgb) in tire_meshes(
                v["wheel_radius"], v["wheel_width"]
            ).items():
                visual = ET.SubElement(wheel, "visual", name=f"tire_{part}")
                geometry = ET.SubElement(visual, "geometry")
                ET.SubElement(geometry, "mesh", filename=f"meshes/tire_{part}.obj")
                material = ET.SubElement(visual, "material", name=f"tire_{part}")
                ET.SubElement(material, "color", rgba=vector([*rgb, 1]))
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
    if "bodies" in p:
        apply_reconstructed_bodies(root, p, output)
    if "cylinder_bodies" in p:
        apply_cylinder_joints(root, p)
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
    if "cylinder_bodies" in p:
        closure["cylinders"] = [
            {
                "joint": name + "_tip_closure",
                "body0": name + "_rod",
                "body1": spec["tip_body"],
                "local_pos0": [0, 0, 0],
                "local_pos1": spec["tip"],
            }
            for name, spec in p["cylinder_bodies"].items()
        ]
    (output / "closure.json").write_text(json.dumps(closure, indent=2) + "\n")
    return output / "wa320.urdf"


def apply_reconstructed_bodies(root, p, output):
    """Replace all inertias, retain articulation and add explicit fixed component bodies."""
    import numpy as np

    from .mass_properties import body_properties

    props = body_properties(p)
    links = {node.attrib["name"]: node for node in root.findall("link")}
    # Cab has its own mass/geometry; remove duplicated root visuals/collisions.
    for kind in ("visual", "collision"):
        for node in list(links["rear_frame"].findall(kind)):
            links["rear_frame"].remove(node)
    for name, body in p["bodies"].items():
        if name not in links:
            links[name] = ET.SubElement(root, "link", name=name)
            j = ET.SubElement(root, "joint", name=name + "_fixed", type="fixed")
            ET.SubElement(j, "parent", link=body["parent"])
            ET.SubElement(j, "child", link=name)
            ET.SubElement(j, "origin", xyz=vector(body["origin"]))
        link = links[name]
        old = link.find("inertial")
        if old is not None:
            link.remove(old)
        prop = props[name]
        node = ET.SubElement(link, "inertial")
        ET.SubElement(node, "origin", xyz=vector(prop["com_m"]))
        ET.SubElement(node, "mass", value=str(prop["mass_kg"]))
        tensor = np.array(prop["inertia_kg_m2"])
        ET.SubElement(
            node,
            "inertia",
            **{
                key: str(tensor[i, j])
                for key, i, j in [
                    ("ixx", 0, 0),
                    ("iyy", 1, 1),
                    ("izz", 2, 2),
                    ("ixy", 0, 1),
                    ("ixz", 0, 2),
                    ("iyz", 1, 2),
                ]
            },
        )
        # Only rear/axle/cab proxies need new external shapes. Cylinder groups carry
        # inertia but remain non-colliding mass proxies, with replay decorations.
        if name in p["collision_geometry"]:
            for kind in ("visual", "collision"):
                for child in list(link.findall(kind)):
                    link.remove(child)
            for index, primitive in enumerate(p["collision_geometry"][name]):
                shape = primitive["shape"]
                parts = [None]
                if shape == "annulus":
                    # Individual convex sectors retain the bearing hole; one convex
                    # hull of the whole ring would silently fill it in USD import.
                    outer, inner, length = primitive["size"]
                    parts = []
                    for sector in range(p["collision_geometry"]["annulus_segments"]):
                        angles = [
                            2
                            * math.pi
                            * (sector + k)
                            / p["collision_geometry"]["annulus_segments"]
                            for k in (0, 1)
                        ]
                        vertices = [
                            [r * math.cos(a), r * math.sin(a), z]
                            for z in (-length / 2, length / 2)
                            for r, a in [
                                (inner, angles[0]),
                                (outer, angles[0]),
                                (outer, angles[1]),
                                (inner, angles[1]),
                            ]
                        ]
                        mesh = output / "meshes" / f"{name}_{index}_{sector}.obj"
                        faces = [
                            (1, 4, 3, 2),
                            (5, 6, 7, 8),
                            (1, 2, 6, 5),
                            (2, 3, 7, 6),
                            (3, 4, 8, 7),
                            (4, 1, 5, 8),
                        ]
                        mesh.write_text(
                            "# Original convex hinge-ring sector\n"
                            + "".join("v " + vector(v) + "\n" for v in vertices)
                            + "".join(
                                "f " + " ".join(map(str, f)) + "\n" for f in faces
                            )
                        )
                        parts.append(mesh)
                for part in parts:
                    for kind in ("visual", "collision"):
                        child = ET.SubElement(link, kind)
                        ET.SubElement(
                            child,
                            "origin",
                            xyz=vector(primitive["center"]),
                            rpy=vector(primitive["rpy"]),
                        )
                        geo = ET.SubElement(child, "geometry")
                        if part is not None:
                            ET.SubElement(geo, "mesh", filename="meshes/" + part.name)
                        elif shape == "cylinder":
                            ET.SubElement(
                                geo,
                                "cylinder",
                                radius=str(primitive["size"][0]),
                                length=str(primitive["size"][1]),
                            )
                        else:
                            ET.SubElement(geo, "box", size=vector(primitive["size"]))
                        if kind == "visual":
                            ET.SubElement(
                                child,
                                "material",
                                name="glass"
                                if name == "cab"
                                else "steel"
                                if "hinge" in name or name.endswith("_rod")
                                else "yellow",
                            )
    joints = {node.attrib["name"]: node for node in root.findall("joint")}
    for axle in ("front", "rear"):
        for side, sign in (("left", 1), ("right", -1)):
            joint = joints[f"{axle}_{side}_wheel_joint"]
            joint.find("parent").set("link", axle + "_axle")
            joint.find("origin").set(
                "xyz", vector([0, sign * p["vehicle"]["track_width"] / 2, 0])
            )
    c = p["linkage"]["bellcrank_pivot"]
    joints["tilt_joint"].find("origin").set("xyz", vector([c[0], 0, c[1]]))
    d = p["linkage"]["rod_pin_local"]
    joints["rod_joint"].find("origin").set("xyz", vector([d[0], 0, d[1]]))
    # Bucket side-wall extrusion and segmented floor/back leave the mouth open.
    bucket = links["bucket"]
    for kind in ("visual", "collision"):
        for node in list(bucket.findall(kind)):
            bucket.remove(node)
    profile = np.array(p["bucket"]["profile"])
    width, plate = p["vehicle"]["bucket_width"], p["bucket"]["plate_thickness"]
    vertices = [[x, y, z] for y in (-plate / 2, plate / 2) for x, z in profile]
    n = len(profile)
    faces = [list(range(n - 1, -1, -1)), list(range(n, 2 * n))] + [
        [i, (i + 1) % n, (i + 1) % n + n, i + n] for i in range(n)
    ]
    mesh = output / "meshes" / "bucket_side.obj"
    mesh.write_text(
        "# Original approximate bucket side-wall extrusion\n"
        + "".join("v " + vector(v) + "\n" for v in vertices)
        + "".join("f " + " ".join(str(i + 1) for i in face) + "\n" for face in faces)
    )
    for kind in ("visual", "collision"):
        for sign in (-1, 1):
            child = ET.SubElement(bucket, kind)
            ET.SubElement(
                child, "origin", xyz=vector([0, sign * (width - plate) / 2, 0])
            )
            geo = ET.SubElement(child, "geometry")
            ET.SubElement(geo, "mesh", filename="meshes/bucket_side.obj")
            if kind == "visual":
                ET.SubElement(child, "material", name="steel")
        for a, b in itertools.pairwise(profile):
            delta = b - a
            child = ET.SubElement(bucket, kind)
            ET.SubElement(
                child,
                "origin",
                xyz=vector([(a[0] + b[0]) / 2, 0, (a[1] + b[1]) / 2]),
                rpy=vector([0, -math.atan2(delta[1], delta[0]), 0]),
            )
            geo = ET.SubElement(child, "geometry")
            ET.SubElement(
                geo, "box", size=vector([float(np.linalg.norm(delta)), width, plate])
            )
            if kind == "visual":
                ET.SubElement(child, "material", name="steel")


def apply_cylinder_joints(root, p):
    """Pinned barrel -> physical prismatic rod; tip closure is authored in USD."""
    from .mass_properties import body_transforms

    frames = body_transforms(p, 0, 0, 0)
    for name, spec in p["cylinder_bodies"].items():
        for part in ("barrel", "rod"):
            old = root.find(f"joint[@name='{name}_{part}_fixed']")
            root.remove(old)
        _, parent_r = frames[spec["parent"]]
        _, barrel_r = frames[name + "_barrel"]
        r = parent_r.T @ barrel_r
        alongZ = spec["axis"] == "Y"
        angle = math.atan2(r[0, 2], r[2, 2]) if alongZ else math.atan2(r[1, 0], r[0, 0])
        c = p["cylinders"][spec["kind"]]
        for suffix, parent, child, kind, xyz, rpy, axis, lo, hi, effort, speed in [
            (
                "hinge",
                spec["parent"],
                name + "_barrel",
                "revolute",
                spec["base"],
                [0, angle, 0] if alongZ else [0, 0, angle],
                [0, 1, 0] if alongZ else [0, 0, 1],
                -math.pi,
                math.pi,
                p["actuators"][spec["kind"]]["fmax"],
                spec["hinge_speed"],
            ),
            (
                "slider",
                name + "_barrel",
                name + "_rod",
                "prismatic",
                [0, 0, spec["reference_length"]]
                if alongZ
                else [spec["reference_length"], 0, 0],
                [0, 0, 0],
                [0, 0, 1] if alongZ else [1, 0, 0],
                c["pin_distance_min"] - spec["reference_length"],
                c["pin_distance_max"] - spec["reference_length"],
                p["actuators"][spec["kind"]]["fmax"],
                p["actuators"][spec["kind"]]["vmax"],
            ),
        ]:
            j = ET.SubElement(
                root, "joint", name=name + "_" + suffix + "_joint", type=kind
            )
            ET.SubElement(j, "parent", link=parent)
            ET.SubElement(j, "child", link=child)
            ET.SubElement(j, "origin", xyz=vector(xyz), rpy=vector(rpy))
            ET.SubElement(j, "axis", xyz=vector(axis))
            ET.SubElement(
                j,
                "limit",
                lower=str(lo),
                upper=str(hi),
                effort=str(effort),
                velocity=str(speed),
            )
