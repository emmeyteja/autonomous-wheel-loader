"""Isolated Isaac Sim/PhysX foundation checks. Does not implement a training task."""

import json
import math
from pathlib import Path


def run_probe(
    p, usd, output, *, fixture=False, pile=False, capture=False, terrain=None
):
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "width": 1024,
            "height": 768,
            "renderer": "RayTracedLighting",
            "disable_viewport_updates": not capture,
            "multi_gpu": False,
            "active_gpu": 0,
            "physics_gpu": 0,
        }
    )
    exit_code = 0
    try:
        import numpy as np
        import omni.usd
        from isaacsim.core.api import SimulationContext
        from isaacsim.core.experimental.prims import Articulation
        from pxr import Gf, PhysicsSchemaTools, PhysxSchema, UsdGeom, UsdPhysics

        from wheel_loader.control.commands import wheel_rates
        from wheel_loader.terrain.rigid_pile import RigidPile
        from wheel_loader.vehicle.kinematics import passive_joint_positions

        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        (output / "validation.json").unlink(missing_ok=True)
        (output / "error.json").unlink(missing_ok=True)
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        model = UsdGeom.Xform.Define(stage, "/World/Loader")
        model.GetPrim().GetReferences().AddReference(str(Path(usd).resolve()))
        model.AddTranslateOp().Set(
            Gf.Vec3d(
                0, 0, p["vehicle"]["articulation_height"] + (0.5 if fixture else 0.02)
            )
        )
        anchored = any(
            prim.IsA(UsdPhysics.FixedJoint)
            and (
                not UsdPhysics.FixedJoint(prim).GetBody0Rel().GetTargets()
                or not stage.GetPrimAtPath(
                    UsdPhysics.FixedJoint(prim).GetBody0Rel().GetTargets()[0]
                ).HasAPI(UsdPhysics.RigidBodyAPI)
            )
            for prim in stage.Traverse()
        )
        if bool(fixture) != anchored:
            raise ValueError("Fixture mode and fixed/floating USD do not match")
        floor = UsdGeom.Cube.Define(stage, "/World/Floor")
        floor.CreateSizeAttr(1.0)
        floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.1))
        floor.AddScaleOp().Set(Gf.Vec3f(40, 40, 0.2))
        floor.CreateDisplayColorAttr([Gf.Vec3f(0.23, 0.26, 0.28)])
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        if pile:
            obstacle = terrain if terrain is not None else RigidPile()
            mesh = UsdGeom.Mesh.Define(stage, "/World/Pile")
            points = obstacle.vertices()
            points[:, 0] += 8.0
            mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in points])
            mesh.CreateFaceVertexCountsAttr([4] * 6)
            mesh.CreateFaceVertexIndicesAttr(
                [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
            )
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDisplayColorAttr([Gf.Vec3f(0.58, 0.43, 0.22)])
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
                "convexHull"
            )
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                api.CreateSolverPositionIterationCountAttr(
                    p["solver"]["position_iterations"]
                )
                api.CreateSolverVelocityIterationCountAttr(
                    p["solver"]["velocity_iterations"]
                )
                api.CreateEnabledSelfCollisionsAttr(True)
        contact_pairs = {}
        from omni.physx import get_physx_simulation_interface

        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

        def on_contact(headers, contacts):
            for header in headers:
                if not header.num_contact_data:
                    continue
                pair = " | ".join(
                    sorted(
                        [
                            str(PhysicsSchemaTools.intToSdfPath(header.actor0)),
                            str(PhysicsSchemaTools.intToSdfPath(header.actor1)),
                        ]
                    )
                )
                record = contact_pairs.setdefault(
                    pair,
                    {"events": 0, "minimum_separation_m": 0.0, "peak_force_N": 0.0},
                )
                record["events"] += 1
                values = contacts[
                    header.contact_data_offset : header.contact_data_offset
                    + header.num_contact_data
                ]
                for contact in values:
                    record["minimum_separation_m"] = min(
                        record["minimum_separation_m"], float(contact.separation)
                    )
                    record["peak_force_N"] = max(
                        record["peak_force_N"],
                        float(np.linalg.norm(contact.impulse)) / p["solver"]["dt"],
                    )

        subscription = get_physx_simulation_interface().subscribe_contact_report_events(
            on_contact
        )
        sim = SimulationContext(
            physics_dt=p["solver"]["dt"],
            rendering_dt=p["solver"]["dt"],
            stage_units_in_meters=1.0,
            backend="numpy",
        )
        sim.initialize_physics()
        robot = Articulation("/World/Loader")
        robot.set_solver_iteration_counts(
            position_counts=p["solver"]["position_iterations"],
            velocity_counts=p["solver"]["velocity_iterations"],
        )
        robot.set_enabled_self_collisions(True)
        sim.play()
        app.update()
        names = robot.dof_names
        links = robot.link_names
        print("WA320 DOFs:", names, flush=True)
        qtarget = np.zeros((1, len(names)), dtype=np.float32)
        vtarget = np.zeros_like(qtarget)
        if not fixture:
            initial = passive_joint_positions(p, 0.0 if pile else 0.15, -0.3)
            for name, value in initial.items():
                qtarget[0, names.index(name)] = value
            robot.set_dof_positions(qtarget)
            robot.set_dof_velocities(vtarget)
        records = []
        loop_residual = []
        max_tracking = 0.0
        dt = p["solver"]["dt"]
        duration = 12.0 if fixture else (13.0 if pile else 15.0)

        # Backend transform adapter pinned to Isaac Sim 6.2; quaternions are XYZW.
        def point_world(transform, point):
            xyz = transform[:3]
            x, y, z, w = transform[3:]
            matrix = np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ]
            )
            return xyz + matrix @ np.asarray(point)

        def sample():
            q = robot.get_dof_positions().numpy()[0].copy()
            transforms = (
                robot._physics_articulation_view.get_link_transforms().numpy()[0].copy()
            )
            if not np.isfinite(q).all() or not np.isfinite(transforms).all():
                raise RuntimeError("Nonfinite WA320 state")
            rod = point_world(
                transforms[links.index("rod")], [p["linkage"]["rod_length"], 0, 0]
            )
            bx, bz = p["linkage"]["bucket_anchor"]
            bucket = point_world(transforms[links.index("bucket")], [bx, 0, bz])
            return q, transforms, float(np.linalg.norm(rod - bucket))

        for step in range(round(duration / dt)):
            t = step * dt
            if fixture:
                steer = 0.3 * min(1.0, max(0.0, (t - 2) / 2))
                boom = 0.5 * min(1.0, max(0.0, (t - 5) / 2))
                tilt = 0.5 * min(1.0, max(0.0, (t - 8) / 2))
                speed = 0.5 if t < 4 else 0.0
            else:
                steer = 0.0 if pile else 0.3 * min(1.0, max(0.0, (t - 8) / 2))
                boom = 0.0 if pile else 0.15
                tilt = -0.3
                speed = 0.6 * min(1.0, max(0.0, (t - 2) / 2))
            for name, val in [
                ("articulation_joint", steer),
                ("boom_joint", boom),
                ("tilt_joint", tilt),
            ]:
                qtarget[0, names.index(name)] = val
            actual = robot.get_dof_positions().numpy()[0]
            velocities = robot.get_dof_velocities().numpy()[0]
            rates = wheel_rates(
                p,
                speed,
                float(actual[names.index("articulation_joint")]),
                float(velocities[names.index("articulation_joint")]),
            )
            vtarget.fill(0)
            for name, val in rates.items():
                vtarget[0, names.index(name)] = val
            robot.set_dof_position_targets(qtarget)
            robot.set_dof_velocity_targets(vtarget)
            sim.step(render=False)
            if step % 12 == 0:
                q, transforms, residual = sample()
                loop_residual.append(residual)
                rear = transforms[links.index("rear_frame")]
                yaw = math.atan2(
                    2 * (rear[6] * rear[5] + rear[3] * rear[4]),
                    1 - 2 * (rear[4] ** 2 + rear[5] ** 2),
                )
                records.append([t, *q, *rear[:3], yaw])
                # Compare passive geometry at actual driven joint angles, not target angles.
                try:
                    expected = passive_joint_positions(
                        p,
                        float(q[names.index("boom_joint")]),
                        float(q[names.index("tilt_joint")]),
                    )
                    max_tracking = max(
                        max_tracking,
                        abs(
                            float(q[names.index("bucket_joint")])
                            - expected["bucket_joint"]
                        ),
                    )
                except ValueError:
                    # Tiny solver excursions at zero boom are permitted only for FK evaluation.
                    if q[names.index("boom_joint")] < -0.02:
                        raise
        q, transforms, residual = sample()
        data = np.asarray(records)
        qi = lambda name: 1 + names.index(name)
        subscription.unsubscribe()
        passive_control = None
        if fixture:
            disturbed = q.copy()
            disturbed[names.index("rod_joint")] += 0.2
            robot.set_dof_positions(disturbed[None, :])
            robot.set_dof_velocities(np.zeros_like(qtarget))
            _, _, before = sample()
            for _ in range(round(2 / dt)):
                sim.step(render=False)
            _, _, after = sample()
            passive_control = {
                "initial_gap_m": before,
                "recovered_gap_m": after,
                "passed": bool(before > 0.1 and after < 0.005),
            }
        limits_ok = all(
            data[:, qi(joint)].min() >= p[section]["min_angle"] - 0.005
            and data[:, qi(joint)].max() <= p[section]["max_angle"] + 0.005
            for joint, section in [
                ("articulation_joint", "articulation"),
                ("boom_joint", "boom"),
                ("tilt_joint", "linkage"),
                ("bucket_joint", "bucket"),
            ]
        )
        checks = {
            "nominal_joint_limits": bool(limits_ok),
            "finite_states": bool(np.isfinite(data).all()),
            "loop_closure": bool(max(loop_residual) < 0.005),
            "wheel_rotation": bool(
                all(
                    np.ptp(data[:, qi(name)]) > 0.5
                    for name in names
                    if name.endswith("wheel_joint")
                )
            ),
            "articulation_motion": bool(np.ptp(data[:, qi("articulation_joint")]) > 0.2)
            if not pile
            else None,
            "boom_motion": bool(np.ptp(data[:, qi("boom_joint")]) > 0.35)
            if fixture
            else None,
            "passive_bucket_motion": bool(np.ptp(data[:, qi("bucket_joint")]) > 0.2)
            if fixture
            else None,
            "passive_fk": bool(max_tracking < 0.03),
            "closure_positive_control": passive_control["passed"] if fixture else None,
            "no_deep_self_penetration": not any(
                pair.count("/World/Loader") == 2
                and record["minimum_separation_m"] < -0.002
                for pair, record in contact_pairs.items()
            ),
        }
        if not fixture:
            checks["driving"] = bool(data[-1, -4] - data[0, -4] > 0.5)
            if pile:
                checks["bucket_pile_contact"] = any(
                    "/World/Pile" in pair
                    and "/bucket" in pair
                    and value["peak_force_N"] > 1.0
                    for pair, value in contact_pairs.items()
                )
            if not pile:
                straight = data[(data[:, 0] >= 2) & (data[:, 0] < 8)]
                checks["straight_heading"] = bool(
                    np.max(np.abs(straight[:, -1])) < 0.05
                )
                checks["straight_lateral_drift"] = bool(np.ptp(straight[:, -3]) < 0.1)
                turning = data[data[:, 0] >= 11]
                checks["ground_turning"] = bool(turning[-1, -1] - turning[0, -1] > 0.05)
            checks["rigid_contact_penetration"] = not any(
                "/World/Floor" in pair and value["minimum_separation_m"] < -0.01
                for pair, value in contact_pairs.items()
            )
        result = {
            "runtime": "Isaac Sim 6.2 / PhysX",
            "vehicle": "wa320-like original estimated geometry",
            "fixture": fixture,
            "rigid_pile": pile,
            "checks": checks,
            "maximum_loop_gap_m": max(loop_residual),
            "maximum_passive_fk_error_rad": max_tracking,
            "displacement_m": float(data[-1, -4] - data[0, -4]),
            "yaw_change_rad": float(data[-1, -1] - data[0, -1]),
            "contact_pairs": contact_pairs,
            "closure_positive_control": passive_control,
            "physical_accuracy": "not established",
            "joint_names": names,
            "joint_min_rad": data[:, 1 : 1 + len(names)].min(0).tolist(),
            "joint_max_rad": data[:, 1 : 1 + len(names)].max(0).tolist(),
        }
        np.savez_compressed(
            output / "motion.npz",
            samples=data,
            joint_names=np.asarray(names),
            loop_gap_m=loop_residual,
        )
        (output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
        if capture:
            sim.pause()
            capture_image(stage, app, output, transforms[links.index("rear_frame"), :3])
        if not all(value for value in checks.values() if value is not None):
            raise RuntimeError("WA320 foundation checks failed; see validation.json")
        return result
    except BaseException:  # noqa: BLE001 - Kit close exits the process; preserve failures.
        import traceback

        exit_code = 1
        traceback.print_exc()
    finally:
        app.close(exit_code=exit_code)


def capture_image(stage, app, output, position):
    """Verification image only; no camera observations or perception pipeline."""
    import numpy as np
    import omni.replicator.core as rep
    from PIL import Image
    from pxr import Gf, UsdGeom, UsdLux

    dome = UsdLux.DomeLight.Define(stage, "/World/Light")
    dome.CreateIntensityAttr(700.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(1800.0)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-40, 30, 0))
    camera = UsdGeom.Camera.Define(stage, "/World/VerificationView")
    center = Gf.Vec3d(float(position[0]) + 1.0, float(position[1]), 1.5)
    camera.AddTransformOp().Set(
        Gf.Matrix4d()
        .SetLookAt(center + Gf.Vec3d(9, -13, 7), center, Gf.Vec3d(0, 0, 1))
        .GetInverse()
    )
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 200))
    camera.CreateFocalLengthAttr(35.0)
    product = rep.create.render_product(str(camera.GetPath()), (1024, 768))
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach([product])
    rep.orchestrator.set_capture_on_play(False)
    for _ in range(30):
        app.update()
    rep.orchestrator.step(rt_subframes=16, pause_timeline=True)
    pixels = np.asarray(annotator.get_data())
    if pixels.ndim != 3 or pixels[:, :, :3].std() < 5:
        raise RuntimeError("Empty verification render")
    Image.fromarray(pixels[:, :, :3]).save(output / "wa320_isaacsim.png")
