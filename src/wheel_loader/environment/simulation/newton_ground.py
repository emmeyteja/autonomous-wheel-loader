"""Floating Newton rigid-ground diagnostic, separate from a training task."""

import json
from dataclasses import asdict
from pathlib import Path

import numpy as np


def contact_sweep(path, positions, roots, names, bucket_forces=None):
    """Re-evaluate recorded floating poses using the exported collision model."""
    import mujoco

    if bucket_forces is not None:
        bucket_forces = np.asarray(bucket_forces)
        if (
            len(bucket_forces) != len(positions)
            or bucket_forces.ndim not in (2, 3)
            or bucket_forces.shape[-1] != 3
            or not np.isfinite(bucket_forces).all()
        ):
            raise ValueError(
                "Bucket forces must be finite XYZ samples aligned with recorded positions"
            )
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    joint_ids = {
        (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or "").split(
            "_Physics_"
        )[-1]: i
        for i in range(model.njnt)
    }
    free = np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
    if len(free) != 1:
        raise ValueError("Ground validation requires exactly one floating root")
    base = model.jnt_qposadr[free[0]]
    addresses = [model.jnt_qposadr[joint_ids[name]] for name in names]
    pairs = {}
    wheel_contacts = set()
    supported_samples = 0
    bucket_pile_indices = set()
    for sample, (q, pose) in enumerate(zip(positions, roots)):
        data.qpos[addresses] = q
        data.qpos[base : base + 3] = pose[:3]
        data.qpos[base + 3 : base + 7] = pose[[6, 3, 4, 5]]  # XYZW -> WXYZ
        mujoco.mj_forward(model, data)
        sample_wheels = set()
        for contact in data.contact[: data.ncon]:
            bodies = [int(model.geom_bodyid[int(geom)]) for geom in contact.geom]
            labels = [
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
                for body in bodies
            ]
            key = " | ".join(sorted(labels))
            record = pairs.setdefault(key, {"samples": 0, "minimum_separation_m": 0.0})
            record["samples"] += 1
            record["minimum_separation_m"] = min(
                record["minimum_separation_m"], float(contact.dist)
            )
            geom_names = [
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom)) or ""
                for geom in contact.geom
            ]
            is_pile = any("Pile" in name for name in geom_names)
            if (
                is_pile
                and contact.dist <= 0
                and any("bucket" in name for name in labels)
            ):
                bucket_pile_indices.add(sample)
            if 0 in bodies and not is_pile and contact.dist <= 0:
                sample_wheels.update(name for name in labels if "wheel" in name)
        wheel_contacts.update(sample_wheels)
        supported_samples += len(sample_wheels) == 4
    return {
        "bucket_pile_contact_samples": len(bucket_pile_indices),
        "peak_bucket_force_at_pile_contact_N": float(
            max(
                (np.linalg.norm(bucket_forces[i]) for i in bucket_pile_indices),
                default=0.0,
            )
        )
        if bucket_forces is not None
        else None,
        "pairs": pairs,
        "wheel_bodies_contacting_ground": sorted(wheel_contacts),
        "four_wheels_contact_ground": len(wheel_contacts) == 4,
        "four_wheel_nonpositive_distance_fraction": supported_samples / len(positions),
        "ground_penetration": all(
            record["minimum_separation_m"] >= -0.01
            for pair, record in pairs.items()
            if "world" in pair.split(" | ")
        ),
        "self_penetration": all(
            record["minimum_separation_m"] >= -0.002
            for pair, record in pairs.items()
            if "world" not in pair.split(" | ")
        ),
    }


def run_ground(
    p, usd, output, *, profile, contact_profile=None, scenario=None, pile=False
):
    import isaaclab.sim as sim_utils
    import torch
    from isaaclab.app import launch_simulation
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.utils import configclass
    from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg
    from pxr import Gf, Sdf, Usd, UsdPhysics

    from wheel_loader.control.commands import LoaderCommand, wheel_rates
    from wheel_loader.control.v_cycle import PHASES, VCycleController
    from wheel_loader.evaluation.rigid_ground import (
        evaluate_ground,
        evaluate_wheel_support,
    )
    from wheel_loader.evaluation.v_cycle import evaluate_v_cycle
    from wheel_loader.vehicle.kinematics import passive_joint_positions

    from .fixture_checks import ground_reset_check, linkage_gap
    from .ground_contact import (
        GroundContactProfile,
        audit_ground_model,
        author_tire_contacts,
    )

    if pile and scenario is None:
        raise ValueError("Rigid pile requires a complete V-cycle scenario")
    contact_profile = contact_profile or GroundContactProfile()
    from .isaaclab_probe import articulation_config
    from .newton_profile import audit_import, prepare_asset

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "validation.json").unlink(missing_ok=True)
    dt = p["solver"]["dt"]
    profile.validate_timestep(dt)
    usd = prepare_asset(usd, output / "wa320_newton.usda", profile)
    stage = Usd.Stage.Open(str(usd))
    if any(prim.IsA(UsdPhysics.FixedJoint) for prim in stage.Traverse()):
        raise ValueError("Ground diagnostic requires the floating asset")

    from pxr import UsdGeom, UsdShade

    author_tire_contacts(stage, p, contact_profile)

    floor_path = output / "ground.usda"
    floor_stage = Usd.Stage.CreateNew(str(floor_path.resolve()))
    floor_root = UsdGeom.Xform.Define(floor_stage, "/Ground")
    floor_stage.SetDefaultPrim(floor_root.GetPrim())
    UsdGeom.SetStageUpAxis(floor_stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(floor_stage, 1.0)
    plane = UsdGeom.Plane.Define(floor_stage, "/Ground/CollisionPlane")
    plane.CreateAxisAttr("Z")
    plane.GetPrim().CreateAttribute("mjc:solref", Sdf.ValueTypeNames.Float2).Set(
        Gf.Vec2f(contact_profile.contact_time_constant, contact_profile.damping_ratio)
    )
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    material = UsdShade.Material.Define(floor_stage, "/Ground/Material")
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(p["contact"]["static_friction"])
    physics.CreateDynamicFrictionAttr(p["contact"]["dynamic_friction"])
    physics.CreateRestitutionAttr(p["contact"]["restitution"])
    UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(
        material, materialPurpose="physics"
    )
    obstacle = None
    if pile:
        from .rigid_obstacle import author_pile

        obstacle = author_pile(floor_stage, scenario, material, contact_profile)
    floor_stage.GetRootLayer().Save()

    loader = articulation_config(p, usd)
    loader.spawn.activate_contact_sensors = True
    loader.init_state.pos = (0, 0, p["vehicle"]["articulation_height"] + 0.02)
    loader.init_state.joint_pos = passive_joint_positions(p, 0.15, -0.3)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(
            prim_path="/World/Ground",
            spawn=sim_utils.UsdFileCfg(usd_path=str(floor_path.resolve())),
        )

    scene_cfg = SceneCfg(num_envs=1, env_spacing=12.0)
    scene_cfg.loader = loader
    scene_cfg.wheel_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Loader/Geometry/rear_frame/.*wheel",
        update_period=0.0,
    )
    if pile:
        scene_cfg.bucket_contacts = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Loader/Geometry/rear_frame/front_frame/boom/bucket",
            update_period=0.0,
        )
    cfg = sim_utils.SimulationCfg(
        dt=dt,
        device="cuda:0",
        visualizer_cfgs=[],
        physics=NewtonCfg(
            num_substeps=profile.num_substeps,
            solver_cfg=MJWarpSolverCfg(
                solver=contact_profile.solver,
                cone=contact_profile.cone,
                integrator="implicitfast",
                iterations=profile.iterations,
                njmax=profile.njmax,
                nconmax=profile.nconmax,
                save_to_mjcf=str(output / "effective_model.xml"),
            ),
        ),
    )
    samples, roots, gaps, forces = [], [], [], []
    phase_ids, commands, bucket_forces = [], [], []
    cycle = VCycleController(p, scenario) if scenario else None
    with launch_simulation(cfg):
        sim = sim_utils.SimulationContext(cfg)
        try:
            scene = InteractiveScene(scene_cfg)
            sim.reset()
            robot = scene["loader"]
            names = robot.joint_names
            target = torch.zeros((1, len(names)), device=sim.device)
            velocity = torch.zeros_like(target)
            for name, value in passive_joint_positions(p, 0.15, -0.3).items():
                target[0, names.index(name)] = value
            # Newton 3's import initializes these joints at zero despite init_state.
            # Write a closed, consistent pose before gravity/contact integration.
            robot.write_joint_position_to_sim_index(position=target)
            robot.write_joint_velocity_to_sim_index(velocity=velocity)
            robot.reset()
            initial_error = float(
                torch.max(torch.abs(robot.data.joint_pos.torch - target))
            )
            if initial_error > 1e-6:
                raise RuntimeError("Ground initial joint pose was not applied")
            for step in range(round((scenario.max_duration if scenario else 15) / dt)):
                t = step * dt
                command = LoaderCommand(
                    rear_axle_speed=0.6 * min(1.0, max(0.0, (t - 2) / 2)),
                    articulation_angle=0.3 * min(1.0, max(0.0, (t - 8) / 2)),
                    boom_angle=0.15,
                    tilt_angle=-0.3,
                )
                if cycle:
                    root = robot.data.root_link_pose_w.torch[0].cpu().numpy()
                    x, y, z, w = root[3:]
                    yaw = float(
                        np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                    )
                    phase_index = cycle.phase
                    command = cycle.command(
                        t,
                        dt,
                        root[:2],
                        yaw,
                        articulation=float(
                            robot.data.joint_pos.torch[
                                0, names.index("articulation_joint")
                            ]
                        ),
                    )
                    phase_ids.append(phase_index)
                    if cycle.phase != phase_index:
                        print(
                            f"V-cycle: {PHASES[phase_index]} complete at {t:.1f}s",
                            flush=True,
                        )
                commands.append(
                    [
                        command.rear_axle_speed,
                        command.articulation_angle,
                        command.boom_angle,
                        command.tilt_angle,
                    ]
                )
                command.validate(p)
                target[0, names.index("boom_joint")] = command.boom_angle
                target[0, names.index("tilt_joint")] = command.tilt_angle
                target[0, names.index("articulation_joint")] = (
                    command.articulation_angle
                )
                q = robot.data.joint_pos.torch[0]
                qd = robot.data.joint_vel.torch[0]
                rates = wheel_rates(
                    p,
                    command.rear_axle_speed,
                    float(q[names.index("articulation_joint")]),
                    float(qd[names.index("articulation_joint")]),
                )
                for name, value in rates.items():
                    velocity[0, names.index(name)] = value
                robot.actuators.target_command.set_position_index(value=target)
                robot.actuators.target_command.set_velocity_index(value=velocity)
                robot.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)
                q = robot.data.joint_pos.torch[0].cpu().numpy().copy()
                pose = (
                    robot.data.body_link_pose_w.torch[
                        0, robot.body_names.index("rear_frame")
                    ]
                    .cpu()
                    .numpy()
                    .copy()
                )
                if not np.isfinite(q).all() or not np.isfinite(pose).all():
                    raise RuntimeError("Nonfinite floating-ground state")
                samples.append(q)
                roots.append(pose)
                gaps.append(linkage_gap(robot, p))
                forces.append(
                    scene["wheel_contacts"]
                    .data.net_forces_w.torch[0]
                    .cpu()
                    .numpy()
                    .copy()
                )
                if pile:
                    bucket_forces.append(
                        scene["bucket_contacts"]
                        .data.net_forces_w.torch[0]
                        .cpu()
                        .numpy()
                        .copy()
                    )
                if cycle and cycle.done:
                    break
            positions, roots = np.asarray(samples), np.asarray(roots)
            times = np.arange(len(positions)) * dt
            evaluation = (
                evaluate_v_cycle(
                    p,
                    asdict(scenario) | {"joint_names": names},
                    positions,
                    roots,
                    phase_ids,
                    commands,
                    cycle.events,
                    cycle.done and not cycle.timeout,
                )
                if cycle
                else evaluate_ground(p, times, positions, roots, gaps, names)
            )
            contacts = contact_sweep(
                output / "effective_model.xml",
                positions,
                roots,
                names,
                np.asarray(bucket_forces) if pile else None,
            )
            forces = np.asarray(forces)
            support = evaluate_wheel_support(times, forces)
            checks = (
                audit_import(output / "effective_model.xml", profile)
                | audit_ground_model(output / "effective_model.xml", contact_profile)
                | evaluation["checks"]
                | {
                    "loop_closure": bool(max(gaps) < 0.005),
                    "four_wheels_contact_ground": contacts[
                        "four_wheels_contact_ground"
                    ],
                    "sustained_wheel_support": support["sustained_wheel_support"],
                    "rigid_contact_penetration": contacts["ground_penetration"],
                    "self_collision": contacts["self_penetration"],
                }
            )
            if pile:
                checks["bucket_contacts_rigid_pile"] = (
                    contacts["bucket_pile_contact_samples"] > 0
                )
                checks["bucket_pile_measured_force"] = (
                    contacts["peak_bucket_force_at_pile_contact_N"] > 1.0
                )
            reset_check = ground_reset_check(robot, scene, sim, p, dt)
            checks["floating_reset_repeatability"] = reset_check["passed"]
            result = {
                "runtime": "Isaac Lab 3 / Newton / MuJoCo Warp",
                "fixture": False,
                "rigid_pile": obstacle,
                "scenario": asdict(scenario) if scenario else None,
                "initial_joint_error_rad": initial_error,
                "floating_reset": reset_check,
                "numerical_profile": asdict(profile),
                "checks": checks,
                **{key: value for key, value in evaluation.items() if key != "checks"},
                "maximum_loop_gap_m": float(max(gaps)),
                "contacts": contacts,
                **{
                    key: value
                    for key, value in support.items()
                    if key != "sustained_wheel_support"
                },
                "ground_collision": {
                    "enable_multiccd": False,
                    **asdict(contact_profile),
                    "floor_geometry": "analytic Z-up plane",
                },
                "force_sensor_body_names": scene["wheel_contacts"].body_names,
                "physical_accuracy": "not established",
            }
            np.savez_compressed(
                output / "motion.npz",
                positions=positions,
                root_poses_xyzw=roots,
                joint_names=np.asarray(names),
                loop_gap_m=gaps,
                time_s=times,
                wheel_contact_forces_w=forces,
                bucket_contact_forces_w=np.asarray(bucket_forces),
                phase_ids=np.asarray(phase_ids),
                phase_names=np.asarray(PHASES if cycle else []),
                commands=np.asarray(commands),
            )
            (output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2), flush=True)
            if not all(checks.values()):
                raise RuntimeError(
                    "Newton rigid-ground checks failed; see validation.json"
                )
            return result
        finally:
            sim_utils.SimulationContext.clear_instance()
