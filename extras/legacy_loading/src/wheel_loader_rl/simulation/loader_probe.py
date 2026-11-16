"""Author the small primitive loader directly in USD; no Isaac Sim importer needed."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET


def author_loader(output):
    """Translate only this project's box/cylinder/capsule asset, not arbitrary MJCF."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
    output.mkdir(parents=True, exist_ok=True)
    path = output.resolve() / 'loader_approximation.usda'
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, '/Loader')
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    material = UsdShade.Material.Define(stage, '/Loader/Material')
    friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    friction.CreateStaticFrictionAttr(0.55)
    friction.CreateDynamicFrictionAttr(0.55)
    friction.CreateRestitutionAttr(0.0)
    source = ET.parse(Path(__file__).parents[1] / 'assets/loader_approximation.xml').getroot()
    inertia = {'chassis': (4000, 10000, 10000), 'lift_arm': (20, 250, 250), 'bucket': (100, 150, 200)}

    def numbers(element, name, default='0 0 0'):
        return tuple(float(v) for v in element.get(name, default).split())

    def body(element, parent_path, parent_world):
        name = element.get('name')
        body_path = '/Loader/' + name
        local = numbers(element, 'pos')
        world = tuple(a + b for a, b in zip(parent_world, local))
        link = UsdGeom.Xform.Define(stage, body_path)
        # Flat rigid-body hierarchy avoids nested rigid-body transformations.
        link.AddTranslateOp().Set(Gf.Vec3d(*world))
        UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(link.GetPrim())
        mass.CreateMassAttr(sum(float(g.get('mass')) for g in element.findall('geom')))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia[name]))
        for geom in element.findall('geom'):
            shape_path = body_path + '/' + geom.get('name')
            kind, size = geom.get('type'), numbers(geom, 'size')
            if kind == 'box':
                shape = UsdGeom.Cube.Define(stage, shape_path)
                shape.CreateSizeAttr(2.0)
                shape.AddTranslateOp().Set(Gf.Vec3d(*numbers(geom, 'pos')))
                shape.AddScaleOp().Set(Gf.Vec3f(*size))
            elif kind == 'cylinder':
                shape = UsdGeom.Cylinder.Define(stage, shape_path)
                shape.CreateRadiusAttr(size[0]); shape.CreateHeightAttr(2 * size[1]); shape.CreateAxisAttr('Y')
                shape.AddTranslateOp().Set(Gf.Vec3d(*numbers(geom, 'pos')))
            elif kind == 'capsule':
                endpoints = numbers(geom, 'fromto')
                a, b = Gf.Vec3d(*endpoints[:3]), Gf.Vec3d(*endpoints[3:])
                shape = UsdGeom.Capsule.Define(stage, shape_path)
                shape.CreateRadiusAttr(size[0]); shape.CreateHeightAttr((b - a).GetLength())
                shape.AddTranslateOp().Set((a + b) / 2)
                shape.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), (b-a).GetNormalized()).GetQuat()))
            else:
                raise ValueError(f'Unsupported authored loader shape: {kind}')
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material, materialPurpose='physics')
            shape.CreateDisplayColorAttr([Gf.Vec3f(*numbers(geom, 'rgba', '0.9 0.6 0.08 1')[:3])])
        spec = element.find('joint')
        cls = UsdPhysics.PrismaticJoint if spec.get('type') == 'slide' else UsdPhysics.RevoluteJoint
        joint = cls.Define(stage, '/Loader/' + spec.get('name'))
        joint.CreateBody1Rel().SetTargets([body_path])
        if parent_path:
            joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(local if parent_path else world)))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        slide = spec.get('type') == 'slide'
        joint.CreateAxisAttr('X' if slide else 'Y')
        if not slide:
            # Both joint frames rotate +Y onto -Y, matching the asset convention.
            joint.CreateLocalRot0Attr(Gf.Quatf(0, 1, 0, 0))
            joint.CreateLocalRot1Attr(Gf.Quatf(0, 1, 0, 0))
        lower, upper = numbers(spec, 'range')
        joint.CreateLowerLimitAttr(lower if slide else math.degrees(lower))
        joint.CreateUpperLimitAttr(upper if slide else math.degrees(upper))
        joint.CreateCollisionEnabledAttr(False)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear' if slide else 'angular')
        drive.CreateTypeAttr('force'); drive.CreateStiffnessAttr(200000); drive.CreateDampingAttr(30000)
        drive.CreateMaxForceAttr(100000)
        for child in element.findall('body'):
            body(child, body_path, world)

    body(source.find('worldbody/body'), None, (0, 0, 0))
    stage.GetRootLayer().Save()
    return path


def loader_config(output):
    import isaaclab.sim as sim_utils
    from isaaclab.assets import ArticulationCfg
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_newton.sim.schemas import NewtonArticulationCfg
    return ArticulationCfg(
        prim_path='{ENV_REGEX_NS}/Loader',
        spawn=sim_utils.UsdFileCfg(usd_path=str(author_loader(output)),
            articulation_props=[NewtonArticulationCfg(self_collision_enabled=False)]),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos={'travel': 0.0, 'lift': 0.0, 'curl': 0.0}),
        actuators={
            'travel': ImplicitActuatorCfg(joint_names_expr=['travel'], joint_effort_limit=100000,
                                          stiffness=200000, damping=40000),
            'attachment': ImplicitActuatorCfg(joint_names_expr=['lift', 'curl'], joint_effort_limit=100000,
                                              stiffness=2000000, damping=60000),
        },
    )
