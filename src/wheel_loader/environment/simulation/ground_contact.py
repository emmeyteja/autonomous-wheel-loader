"""Explicit Newton ground-contact approximation; not measured tire compliance."""

import math
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class GroundContactProfile:
    solver: str = "newton"
    cone: str = "elliptic"
    tire_geometry: str = "cylinder"
    contact_time_constant: float = 0.04
    damping_ratio: float = 1.0

    def __post_init__(self):
        if self.solver not in ("cg", "newton") or self.cone not in (
            "pyramidal",
            "elliptic",
        ):
            raise ValueError("Unsupported ground solver or friction cone")
        if self.tire_geometry not in ("hull", "cylinder"):
            raise ValueError("Unsupported tire contact geometry")
        for value in (self.contact_time_constant, self.damping_ratio):
            if (
                isinstance(value, bool)
                or not isinstance(value, (float, int))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(
                    "Contact reference time and damping ratio must be finite and positive"
                )


def load_ground_profile(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise TypeError("Ground-contact profile must be a mapping")
    if set(data) != set(GroundContactProfile.__dataclass_fields__):
        raise ValueError("Unsupported ground-contact profile fields")
    return GroundContactProfile(**data)


def author_tire_contacts(stage, p, profile):
    """Modify the derived Newton stage only; preserve body mass/inertia and drives."""
    import numpy as np
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    tires = [prim for prim in stage.Traverse() if prim.GetName() == "tire_contact"]
    if len(tires) != 4:
        raise ValueError("Expected four original tire contact hulls")
    radius, width = p["vehicle"]["wheel_radius"], p["vehicle"]["wheel_width"]
    for prim in tires:
        if not prim.IsA(UsdGeom.Mesh):
            raise ValueError("Expected original mesh contact geometry")
        points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
        expected = np.array([radius, width / 2, radius])
        if not np.allclose(points.min(axis=0), -expected, atol=1e-5) or not np.allclose(
            points.max(axis=0), expected, atol=1e-5
        ):
            raise ValueError("Tire hull dimensions differ from configured dimensions")
        if profile.tire_geometry == "cylinder":
            prim.SetTypeName("Cylinder")
            prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
            cylinder = UsdGeom.Cylinder(prim)
            cylinder.CreateAxisAttr("Y")
            cylinder.CreateRadiusAttr(radius)
            cylinder.CreateHeightAttr(width)
        prim.CreateAttribute("mjc:solref", Sdf.ValueTypeNames.Float2).Set(
            Gf.Vec2f(profile.contact_time_constant, profile.damping_ratio)
        )
    stage.GetRootLayer().Save()


def audit_ground_model(path, profile):
    import mujoco
    import numpy as np

    model = mujoco.MjModel.from_xml_path(str(path))
    tires = [
        i
        for i in range(model.ngeom)
        if "tire_contact"
        in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "")
    ]
    expected_type = (
        mujoco.mjtGeom.mjGEOM_CYLINDER
        if profile.tire_geometry == "cylinder"
        else mujoco.mjtGeom.mjGEOM_MESH
    )
    static_contacts = [i for i in range(model.ngeom) if model.geom_bodyid[i] == 0]
    return {
        "ground_solver_consumed": int(model.opt.solver)
        == int(
            mujoco.mjtSolver.mjSOL_NEWTON
            if profile.solver == "newton"
            else mujoco.mjtSolver.mjSOL_CG
        ),
        "friction_cone_consumed": int(model.opt.cone)
        == int(
            mujoco.mjtCone.mjCONE_ELLIPTIC
            if profile.cone == "elliptic"
            else mujoco.mjtCone.mjCONE_PYRAMIDAL
        ),
        "tire_geometry_consumed": len(tires) == 4
        and all(model.geom_type[i] == expected_type for i in tires),
        "contact_reference_consumed": bool(
            len(static_contacts) >= 1
            and len(tires) == 4
            and np.allclose(
                model.geom_solref[tires + static_contacts],
                [profile.contact_time_constant, profile.damping_ratio],
                rtol=1e-5,
            )
        ),
    }
