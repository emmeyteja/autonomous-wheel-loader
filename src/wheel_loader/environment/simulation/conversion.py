"""Supported Isaac Sim importer followed by an explicit passive Z-bar loop joint."""

import json
import math
from pathlib import Path


def convert(p, urdf: Path, output: Path, *, fixed=False):
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    wheel_names = [
        f"{axle}_{side}_wheel_joint"
        for axle in ("rear", "front")
        for side in ("left", "right")
    ]
    driven = ["articulation_joint", "boom_joint", "tilt_joint"]
    passive = ["bucket_joint", "rod_joint"]
    passive += [
        name + "_" + part + "_joint"
        for name in p.get("cylinder_bodies", {})
        for part in ("hinge", "slider")
    ]
    target = (
        dict.fromkeys(wheel_names, "velocity")
        | dict.fromkeys(driven, "position")
        | dict.fromkeys(passive, "none")
    )
    # Importer gain overrides use SI; USD stores angular gains per degree.
    stiffness = dict.fromkeys(wheel_names + passive, 0.0) | dict.fromkeys(
        driven, p["solver"]["joint_stiffness"]
    )
    damping = (
        dict.fromkeys(wheel_names, p["solver"]["wheel_damping"])
        | dict.fromkeys(driven, p["solver"]["joint_damping"])
        | dict.fromkeys(passive, 0.0)
    )
    if "actuators" in p:
        stiffness = dict.fromkeys(wheel_names + driven + passive, 0.0)
        damping = dict.fromkeys(wheel_names + driven + passive, 0.0)
    config = URDFImporterConfig(
        urdf_path=str(Path(urdf).resolve()),
        usd_path=str(Path(output).resolve()),
        merge_fixed_joints=False,
        merge_mesh=False,
        collision_from_visuals=False,
        collision_type="Convex Hull",
        allow_self_collision=True,
        fix_base=fixed,
        joint_drive_type="force",
        joint_target_type=target,
        override_joint_stiffness=stiffness,
        override_joint_damping=damping,
        run_asset_transformer=True,
        run_multi_physics_conversion=True,
    )
    result = Path(URDFImporter(config).import_urdf())
    stage = Usd.Stage.Open(str(result))
    model = stage.GetDefaultPrim()
    model.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    bodies = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    }
    if not {"rod", "bucket", "rear_frame", "front_frame"} <= set(bodies):
        raise RuntimeError(
            "Converted rigid-body names do not match the closure contract"
        )
    # Explicit tire hull resolution avoids backend-dependent primitive tessellation.
    sides = int(p["wheels"]["collision_sides"])
    if sides < 16 or sides > 128:
        raise ValueError("Tire hull requires 16–128 sides")
    for name, body in bodies.items():
        if not name.endswith("_wheel"):
            continue
        for child in body.GetChildren():
            if child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
        mesh = UsdGeom.Mesh.Define(stage, body.GetPath().AppendChild("tire_contact"))
        radius, width = p["vehicle"]["wheel_radius"], p["vehicle"]["wheel_width"]
        points = [
            Gf.Vec3f(
                radius * math.cos(2 * math.pi * i / sides),
                side * width / 2,
                radius * math.sin(2 * math.pi * i / sides),
            )
            for side in (-1, 1)
            for i in range(sides)
        ]
        faces = [list(range(sides - 1, -1, -1)), list(range(sides, 2 * sides))]
        faces += [
            [i, (i + 1) % sides, (i + 1) % sides + sides, i + sides]
            for i in range(sides)
        ]
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
        mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreatePurposeAttr("guide")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
            "convexHull"
        )
        mesh.GetPrim().AddAppliedSchema("PhysxConvexHullCollisionAPI")
        mesh.GetPrim().CreateAttribute(
            "physxConvexHullCollision:hullVertexLimit", Sdf.ValueTypeNames.Int
        ).Set(2 * sides)
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            section = (
                "wheels"
                if prim.GetName() in wheel_names
                else {
                    "articulation_joint": "articulation",
                    "boom_joint": "boom",
                    "tilt_joint": "linkage",
                }.get(prim.GetName())
            )
            velocity = (
                p[section]["max_velocity"]
                if section
                else p["linkage"]["passive_max_velocity"]
            )
            prim.CreateAttribute(
                "physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float
            ).Set(math.degrees(velocity))
    sidecar = json.loads(Path(urdf).with_name("closure.json").read_text())
    loop = UsdPhysics.RevoluteJoint.Define(
        stage, model.GetPath().AppendChild("zbar_closure")
    )
    loop.CreateBody0Rel().SetTargets([bodies[sidecar["body0"]].GetPath()])
    loop.CreateBody1Rel().SetTargets([bodies[sidecar["body1"]].GetPath()])
    loop.CreateAxisAttr("Y")
    loop.CreateLocalPos0Attr(Gf.Vec3f(*sidecar["local_pos0"]))
    loop.CreateLocalPos1Attr(Gf.Vec3f(*sidecar["local_pos1"]))
    loop.CreateExcludeFromArticulationAttr(True)
    loop.CreateCollisionEnabledAttr(False)
    UsdPhysics.FilteredPairsAPI.Apply(bodies["rod"]).CreateFilteredPairsRel().AddTarget(
        bodies["bucket"].GetPath()
    )
    for spec in sidecar.get("cylinders", []):
        joint = UsdPhysics.SphericalJoint.Define(
            stage, model.GetPath().AppendChild(spec["joint"])
        )
        joint.CreateBody0Rel().SetTargets([bodies[spec["body0"]].GetPath()])
        joint.CreateBody1Rel().SetTargets([bodies[spec["body1"]].GetPath()])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*spec["local_pos0"]))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*spec["local_pos1"]))
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateCollisionEnabledAttr(False)
        UsdPhysics.FilteredPairsAPI.Apply(
            bodies[spec["body0"]]
        ).CreateFilteredPairsRel().AddTarget(bodies[spec["body1"]].GetPath())
    # All passive joints must have no drive, including defaults authored by import.
    for prim in stage.Traverse():
        if prim.GetName() in passive:
            for token in ("angular", "linear"):
                if prim.HasAPI(UsdPhysics.DriveAPI, token):
                    prim.RemoveAPI(UsdPhysics.DriveAPI, token)
    material = UsdShade.Material.Define(
        stage, model.GetPath().AppendChild("RigidContact")
    )
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(p["contact"]["static_friction"])
    physics.CreateDynamicFrictionAttr(p["contact"]["dynamic_friction"])
    physics.CreateRestitutionAttr(p["contact"]["restitution"])
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material, materialPurpose="physics"
            )
    from .tire_visual import author_tire_visuals

    author_tire_visuals(
        stage, p["vehicle"]["wheel_radius"], p["vehicle"]["wheel_width"]
    )
    # Plain URDF bounds are converted by the importer. Ensure passive drives stay absent.
    stage.GetRootLayer().Save()
    # A portable, single-layer PhysX asset: all source geometry is original procedural geometry.
    portable = Path(output) / ("wa320_fixed.usda" if fixed else "wa320.usda")
    flattened = stage.Flatten()
    flattened.documentation = (
        "Original WA320-like parametric asset; estimated geometry and dynamics."
    )
    flattened.Export(str(portable))
    portable.write_text(portable.read_text().rstrip() + "\n")
    metadata = {
        "model": p.get("model_version", "wa320-like original estimated geometry"),
        "physics": "physx",
        "closed_loop": "passive revolute rod-to-bucket; excluded from reduced-coordinate articulation",
        "fixed_base": fixed,
        "mass_kg": p["vehicle"]["mass"],
        "hydraulics": "external first-order force-limited cylinder proxy; gains estimated"
        if "actuators" in p
        else "not modeled; equivalent rotary drives are estimated",
        "physical_accuracy": "unvalidated",
        "usd_file": portable.name,
    }
    (Path(output) / "conversion.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return portable
