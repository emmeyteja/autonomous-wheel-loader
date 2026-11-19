"""Numerical settings applied before Newton imports USD, isolated from vehicle data."""

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

BOUNDED_JOINTS = frozenset(
    {"articulation_joint", "boom_joint", "tilt_joint", "bucket_joint", "rod_joint"}
)


@dataclass(frozen=True)
class NewtonProfile:
    num_substeps: int = 8
    loop_impedance: float = 0.95
    loop_time_constant: float = 0.002  # seconds
    stop_time_constant: float = 0.002  # seconds
    stop_impedance: float = 0.9999  # dimensionless MuJoCo constraint impedance
    iterations: int = 200
    njmax: int = 8192
    nconmax: int = 2048

    def __post_init__(self):
        for name in ("num_substeps", "iterations", "njmax", "nconmax"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        for name in (
            "loop_time_constant",
            "stop_time_constant",
            "stop_impedance",
            "loop_impedance",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (float, int))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")

        if max(self.stop_impedance, self.loop_impedance) > 0.9999:
            raise ValueError(
                "Stop impedance must not exceed MuJoCo's supported maximum .9999"
            )

    def solver_dt(self, control_dt):
        if (
            isinstance(control_dt, bool)
            or not isinstance(control_dt, (int, float))
            or not math.isfinite(control_dt)
            or control_dt <= 0
        ):
            raise ValueError("Control timestep must be finite and positive")
        return control_dt / self.num_substeps

    def validate_timestep(self, control_dt):
        if min(self.loop_time_constant, self.stop_time_constant) < 2 * self.solver_dt(
            control_dt
        ):
            raise ValueError(
                "Constraint reference times must be at least twice the solver timestep"
            )


def load_profile(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise TypeError("Newton profile must be a mapping")
    expected = set(NewtonProfile.__dataclass_fields__)
    if set(data) not in (expected, expected - {"loop_impedance"}):
        raise ValueError("Newton profile fields differ from the supported schema")
    return NewtonProfile(**data)


def prepare_asset(source, output, profile, *, require_fixed_base=False):
    """Derive local USD with explicit Newton stops; leave source/PhysX asset unchanged.

    Raw MuJoCo limit references avoid the engine default force-space stop compliance.
    Scene construction imports the source before sim.reset(), so authoring stops
    after InteractiveScene construction is too late to affect the Newton model.
    """
    from pxr import Gf, Sdf, Usd, UsdPhysics

    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output:
        raise ValueError("Newton preparation must not overwrite its source asset")
    stage = Usd.Stage.Open(str(source))
    if stage is None:
        raise ValueError("Unable to open the source USD")
    root = stage.GetDefaultPrim()
    if not root:
        raise ValueError("Source USD needs a default prim")
    if require_fixed_base:
        anchored = False
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.FixedJoint):
                continue
            joint = UsdPhysics.FixedJoint(prim)
            parent = joint.GetBody0Rel().GetTargets()
            child = joint.GetBody1Rel().GetTargets()
            if (
                child
                and child[0].name == "rear_frame"
                and (
                    not parent
                    or not stage.GetPrimAtPath(parent[0]).HasAPI(
                        UsdPhysics.RigidBodyAPI
                    )
                )
            ):
                anchored = True
        if not anchored:
            raise ValueError("The fixture requires a fixed-base WA320 asset")
    joints = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.RevoluteJoint)
    }
    if not BOUNDED_JOINTS <= joints.keys() or "zbar_closure" not in joints:
        raise ValueError("Source is missing the WA320 joints or passive closure")
    closure = UsdPhysics.RevoluteJoint(joints["zbar_closure"])
    if not closure.GetExcludeFromArticulationAttr().Get():
        raise ValueError(
            "Z-bar closure must be excluded from the reduced-coordinate articulation"
        )
    for name in ("bucket_joint", "rod_joint"):
        if joints[name].HasAPI(UsdPhysics.DriveAPI, "angular"):
            raise ValueError("Passive linkage must not have position/velocity drives")
    output.parent.mkdir(parents=True, exist_ok=True)
    layer = stage.Flatten()
    layer.documentation = (
        "WA320 original geometry with explicit Newton numerical joint stops."
    )
    layer.Export(str(output))
    prepared = Usd.Stage.Open(str(output))
    # Use the same five-DOF hinge closure as Newton's generic importer, but
    # author both connect constraints explicitly so their softness is configurable.
    loop = prepared.GetPrimAtPath(closure.GetPath())
    old = UsdPhysics.RevoluteJoint(loop)
    if old.GetAxisAttr().Get() != "Y":
        raise ValueError("This WA320 profile requires its documented local Y hinge")
    body0, body1 = old.GetBody0Rel().GetTargets(), old.GetBody1Rel().GetTargets()
    pos0, pos1 = old.GetLocalPos0Attr().Get(), old.GetLocalPos1Attr().Get()
    loop.SetActive(False)
    for index, offset in enumerate((0.0, 0.1)):
        joint = UsdPhysics.SphericalJoint.Define(
            prepared, root.GetPath().AppendChild(f"newton_closure_{index}")
        )
        joint.CreateBody0Rel().SetTargets(body0)
        joint.CreateBody1Rel().SetTargets(body1)
        joint.CreateLocalPos0Attr(Gf.Vec3f(pos0[0], pos0[1] + offset, pos0[2]))
        joint.CreateLocalPos1Attr(Gf.Vec3f(pos1[0], pos1[1] + offset, pos1[2]))
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateCollisionEnabledAttr(False)
        prim = joint.GetPrim()
        prim.AddAppliedSchema("MjcEqualityConnectAPI")
        prim.CreateAttribute("mjc:solref", Sdf.ValueTypeNames.Float2).Set(
            Gf.Vec2f(profile.loop_time_constant, 1.0)
        )
        prim.CreateAttribute("mjc:solimp", Sdf.ValueTypeNames.FloatArray).Set(
            [
                0.9 if profile.loop_impedance == 0.95 else profile.loop_impedance,
                profile.loop_impedance,
                0.001,
                0.5,
                2.0,
            ]
        )
    for prim in prepared.Traverse():
        if prim.IsA(UsdPhysics.SphericalJoint) and prim.GetName().endswith(
            "_tip_closure"
        ):
            prim.AddAppliedSchema("MjcEqualityConnectAPI")
            prim.CreateAttribute("mjc:solref", Sdf.ValueTypeNames.Float2).Set(
                Gf.Vec2f(profile.loop_time_constant, 1.0)
            )
            prim.CreateAttribute("mjc:solimp", Sdf.ValueTypeNames.FloatArray).Set(
                [profile.loop_impedance, profile.loop_impedance, 0.001, 0.5, 2.0]
            )
        if (
            prim.IsA(UsdPhysics.PrismaticJoint)
            and prim.GetName().endswith("_slider_joint")
        ) or (prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() in BOUNDED_JOINTS):
            prim.CreateAttribute("mjc:solreflimit", Sdf.ValueTypeNames.Float2).Set(
                Gf.Vec2f(profile.stop_time_constant, 1.0)
            )
            prim.CreateAttribute("mjc:solimplimit", Sdf.ValueTypeNames.FloatArray).Set(
                [profile.stop_impedance, profile.stop_impedance, 0.001, 0.5, 2.0]
            )
    prepared.GetRootLayer().Save()
    return output


def audit_import(path, profile):
    """Verify that the consumer exported the intended limits and passive equality settings."""
    import xml.etree.ElementTree as ET

    root = ET.parse(path).getroot()
    joints = {
        node.attrib["name"].split("_Physics_")[-1]: node
        for node in root.findall(".//joint")
        if "name" in node.attrib
    }

    def close(value, expected):
        parsed = [float(item) for item in value.split()] if value else []
        # MuJoCo's writer omits trailing values equal to the schema defaults.
        defaults = [0.02, 1.0] if len(expected) == 2 else [0.9, 0.95, 0.001, 0.5, 2.0]
        if len(parsed) > len(expected):
            return False
        parsed += defaults[len(parsed) :]
        return all(
            math.isclose(a, b, rel_tol=1e-5, abs_tol=1e-7)
            for a, b in zip(parsed, expected)
        )

    stops = all(
        name in joints
        and close(joints[name].get("solreflimit"), [profile.stop_time_constant, 1.0])
        and close(
            joints[name].get("solimplimit"),
            [profile.stop_impedance, profile.stop_impedance, 0.001, 0.5, 2.0],
        )
        for name in BOUNDED_JOINTS
    )
    equalities = root.findall("equality/connect")
    loops = len(equalities) in (2, 7) and all(
        close(node.get("solref"), [profile.loop_time_constant, 1.0])
        and close(
            node.get("solimp"),
            [
                0.9 if profile.loop_impedance == 0.95 else profile.loop_impedance,
                profile.loop_impedance,
                0.001,
                0.5,
                2.0,
            ],
        )
        for node in equalities
    )
    passive = not any(
        node.get("joint", "").split("_Physics_")[-1] in {"bucket_joint", "rod_joint"}
        for node in root.findall("actuator/*")
    )
    return {
        "stop_settings_consumed": stops,
        "loop_settings_consumed": loops,
        "passive_links_unactuated": passive,
    }
