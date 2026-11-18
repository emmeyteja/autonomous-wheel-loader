"""Apply original tire appearance without touching rigid-body or contact data."""

import xml.etree.ElementTree as ET

from wheel_loader.vehicle.tire_visual import tire_meshes


def author_tire_visuals(stage, radius, width):
    from pxr import Gf, UsdGeom, UsdPhysics

    wheels = [
        p
        for p in stage.Traverse()
        if p.GetName().endswith("_wheel") and p.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    if len(wheels) != 4:
        raise ValueError("Expected four wheel rigid bodies")
    for wheel in wheels:
        for child in wheel.GetChildren():
            # Keep contact shapes available to physics, but out of the beauty view.
            if child.HasAPI(UsdPhysics.CollisionAPI) or not child.GetName().startswith(
                "tire_"
            ):
                UsdGeom.Imageable(child).CreatePurposeAttr("guide")
        for name, (vertices, faces, rgb) in tire_meshes(radius, width).items():
            mesh = UsdGeom.Mesh.Define(
                stage, wheel.GetPath().AppendChild(f"tire_{name}")
            )
            mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in vertices])
            mesh.CreateFaceVertexCountsAttr([3] * len(faces))
            mesh.CreateFaceVertexIndicesAttr([v for f in faces for v in f])
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            mesh.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
            mesh.CreatePurposeAttr("default")


def replay_xml_with_tires(path):
    """Decorate a recorded MuJoCo model in memory; never edit its source XML."""
    root = ET.parse(path).getroot()
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    wheels = [
        body for body in root.iter("body") if body.get("name", "").endswith("_wheel")
    ]
    if len(wheels) != 4:
        raise ValueError("Expected four recorded wheel bodies")
    for body in wheels:
        contacts = [
            g for g in body.findall("geom") if "tire_contact" in g.get("name", "")
        ]
        if len(contacts) != 1 or contacts[0].get("type") != "cylinder":
            raise ValueError(
                "Tire replay expects the recorded analytic cylinder profile"
            )
        radius, half_width = map(float, contacts[0].get("size").split()[:2])
        for geom in body.findall("geom"):
            geom.set("rgba", "0 0 0 0")
        for name, (vertices, faces, rgb) in tire_meshes(radius, 2 * half_width).items():
            mesh_name = body.get("name") + "_visual_" + name
            ET.SubElement(
                asset,
                "mesh",
                name=mesh_name,
                vertex=" ".join(str(v) for point in vertices for v in point),
                face=" ".join(str(v) for face in faces for v in face),
            )
            ET.SubElement(
                body,
                "geom",
                name=mesh_name,
                type="mesh",
                mesh=mesh_name,
                contype="0",
                conaffinity="0",
                mass="0",
                group="2",
                rgba=" ".join(str(v) for v in (*rgb, 1)),
            )
    return ET.tostring(root, encoding="unicode")
