"""Author an immovable diagnostic pile, never transferable material."""


def author_pile(stage, scenario, material, contact_profile):
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    from wheel_loader.terrain.rigid_pile import RigidPile

    shape = RigidPile()
    # Fixed test geometry: leading toe lies 4.85 m beyond the right waypoint.
    center = (scenario.distance + 4.85 + shape.length / 2, -scenario.half_width, 0)
    points = shape.vertices() + center
    mesh = UsdGeom.Mesh.Define(stage, "/Ground/Pile")
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr(
        [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
    )
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.58, 0.43, 0.24)])
    mesh.GetPrim().CreateAttribute("mjc:solref", Sdf.ValueTypeNames.Float2).Set(
        Gf.Vec2f(contact_profile.contact_time_constant, contact_profile.damping_ratio)
    )
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
        "convexHull"
    )
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
        material, materialPurpose="physics"
    )
    return {
        "type": "immovable rigid trapezoidal prism",
        "center_m": center,
        "length_m": shape.length,
        "width_m": shape.width,
        "height_m": shape.height,
        "crest_length_m": shape.crest_length,
        "material_transfer": False,
    }
