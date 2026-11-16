"""Isaac Sim PhysX smoke and RTX screenshots in an isolated optional process."""
from pathlib import Path
import json


def capture(usd_path, output):
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 1024, 'height': 768, 'renderer': 'RayTracedLighting',
                         'anti_aliasing': 3, 'disable_viewport_updates': False})
    try:
        import numpy as np
        from PIL import Image
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
        from isaacsim.core.experimental.prims import Articulation
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        model = UsdGeom.Xform.Define(stage, '/World/Komodo').GetPrim()
        model.GetReferences().AddReference(str(Path(usd_path).resolve()))
        model.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
        scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)
        floor = UsdGeom.Cube.Define(stage, '/World/Floor')
        floor.CreateSizeAttr(1.)
        floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, -.03))
        floor.AddScaleOp().Set(Gf.Vec3f(3, 3, .05))
        floor.CreateDisplayColorAttr([Gf.Vec3f(.18, .22, .27)])
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        dome = UsdLux.DomeLight.Define(stage, '/World/Dome')
        dome.CreateIntensityAttr(400.)
        light = UsdLux.DistantLight.Define(stage, '/World/Sun')
        light.CreateIntensityAttr(1200.)
        light.AddRotateXYZOp().Set(Gf.Vec3f(-40, 30, 0))
        camera = UsdGeom.Camera.Define(stage, '/World/Camera')
        matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(-1.65, -1.65, 1.05), Gf.Vec3d(-.12, 0, .28), Gf.Vec3d(0, 0, 1)).GetInverse()
        camera.AddTransformOp().Set(matrix)
        camera.CreateFocalLengthAttr(35.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 100.))
        product = rep.create.render_product(str(camera.GetPath()), (1024, 768))
        annotator = rep.AnnotatorRegistry.get_annotator('rgb')
        annotator.attach([product])
        rep.orchestrator.set_capture_on_play(False)
        robot = Articulation('/World/Komodo')
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(90):
            app.update()
        # One process owns PhysX and rendering. Isaac Lab/Newton remains untouched.
        names = list(robot.dof_names)
        print('PhysX DOFs:', names, flush=True)
        initial = robot.get_dof_positions().numpy().copy()
        target = initial.copy()
        target[:, names.index('arm_joint')] = .15
        target[:, names.index('bucket_joint')] = .03
        robot.set_dof_position_targets(target)
        velocity = np.zeros_like(target)
        for index, name in enumerate(names):
            if name.endswith('wheel_joint'):
                velocity[:, index] = 1.
        robot.set_dof_velocity_targets(velocity)
        for _ in range(240):
            app.update()
        final = robot.get_dof_positions().numpy().copy()
        timeline.pause()
        for _ in range(40):
            app.update()
        rep.orchestrator.step(rt_subframes=16, pause_timeline=True)
        pixels = np.asarray(annotator.get_data())
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        if pixels.ndim != 3 or pixels.shape[-1] not in (3, 4) or pixels[:, :, :3].std() < 5:
            raise RuntimeError('RTX capture is empty or lacks image content')
        Image.fromarray(pixels[:, :, :3]).save(output/'komodo_isaacsim.png')
        delta = final-initial
        checks = {'wheel_rotation': bool(all(delta[0, i] > .2 for i, name in enumerate(names) if name.endswith('wheel_joint'))),
                  'lift_motion': bool(delta[0, names.index('arm_joint')] > .05),
                  'bucket_motion': bool(delta[0, names.index('bucket_joint')] > .01),
                  'finite_states': bool(np.isfinite(initial).all() and np.isfinite(final).all())}
        result = {'checks': checks, 'runtime': 'Isaac Sim 6.2 / PhysX / RTX', 'variant': 'diagnostic attachment proxies, repaired wheel inertias',
                  'dof_names': names, 'initial_dof_positions_rad': initial.tolist(), 'final_dof_positions_rad': final.tolist(),
                  'all_finite': bool(np.isfinite(final).all()),
                  'warning': 'Smoke/capture only; not full physical or original-geometry validation'}
        (output/'isaacsim_smoke.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2), flush=True)
        if not all(checks.values()):
            raise RuntimeError('Isaac Sim diagnostic motion smoke failed')
    finally:
        app.close()
