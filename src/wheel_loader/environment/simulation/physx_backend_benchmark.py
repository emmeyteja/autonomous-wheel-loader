"""Minimal Isaac Lab/PhysX feasibility replay of the M2 command bundle.

No exact-equivalence claim: MuJoCo rolling/soft contacts and analytic cylinders
are not mapped by this adapter. Numerical/initial-state failures remain failures.
"""

import hashlib
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from .backend_benchmark import Recorder, load_bundle, validate_parameters


def run_physx(bundle, usd, output, parameters, max_ticks=None):
    import isaaclab.sim as sim_utils
    import mujoco
    import torch
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.app import launch_simulation
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab_physx.physics import PhysxCfg

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    m, tape, manifest = load_bundle(bundle)
    validate_parameters(parameters, manifest)
    if np.any(tape["initial_qvel"]):
        raise ValueError(
            "PhysX adapter currently requires the common zero-velocity release"
        )
    if not np.all(m.actuator_gear[:, 0] == 1) or np.any(m.actuator_gear[:, 1:]):
        raise ValueError("PhysX effort mapping requires direct unit actuator gearing")
    if manifest.get("diagnostic", {}).get("case") == "fixed_no_contact":
        raise ValueError(
            "PhysX fixed diagnostic mount is not implemented; use unloaded_ground"
        )
    initial = mujoco.MjData(m)
    initial.qpos[:] = tape["initial_qpos"]
    initial.qvel[:] = tape["initial_qvel"]
    mujoco.mj_forward(m, initial)
    rec = Recorder(
        m,
        initial,
        parameters,
        1 / (manifest["control_dt_s"] * manifest["record_every_ticks"]),
    )
    cfg = sim_utils.SimulationCfg(
        dt=m.opt.timestep,
        device="cpu",
        visualizer_cfgs=[],
        physics=PhysxCfg(solver_type=1),
        gravity=tuple(m.opt.gravity),
    )
    started = time.perf_counter()
    mapping = {
        "status": "NON_EQUIVALENT_MATERIAL_AND_COLLISION_FEASIBILITY_ONLY",
        "unmapped": [
            "MuJoCo solref/solimp",
            "grain rolling resistance 0.0288 m",
            "full six-dimensional torsional/rolling contact law",
            "analytic cylinder collision boundary (PhysX cooks source tire mesh convex hull)",
            "full native collision contype/conaffinity mask mapping beyond explicit body exclusions",
        ],
        "mapped": [
            "source v1 USD masses/inertias and separate bucket pieces",
            "sphere radii/inertias/positions",
            "gravity",
            "physical nine-channel effort tape",
            "240 Hz control / eight substeps",
        ],
        "contact_force_semantics": "last substep contact impulse divided by physics dt",
        "usd_sha256": hashlib.sha256(Path(usd).read_bytes()).hexdigest(),
        "initial_pose_position_tolerance_m": 1e-4,
    }
    (output / "mapping.json").write_text(json.dumps(mapping, indent=2) + "\n")
    with launch_simulation(
        cfg, {"headless": True, "device": "cpu", "enable_cameras": False}
    ):
        import omni.usd
        from pxr import (
            Gf,
            PhysicsSchemaTools,
            PhysxSchema,
            UsdGeom,
            UsdPhysics,
            UsdShade,
        )

        sim = sim_utils.SimulationContext(cfg)
        sim.set_setting("/physics/disableContactProcessing", False)
        stage = omni.usd.get_context().get_stage()
        loader = UsdGeom.Xform.Define(stage, "/World/Loader")
        loader.GetPrim().GetReferences().AddReference(str(Path(usd).resolve()))
        loader.AddTranslateOp().Set(Gf.Vec3d(*initial.xpos[rec.rear]))
        floor = UsdGeom.Plane.Define(stage, "/World/Floor")
        floor.CreateAxisAttr("Z")
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

        def material(path, mu):
            mat = UsdShade.Material.Define(stage, path)
            api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            api.CreateStaticFrictionAttr(mu)
            api.CreateDynamicFrictionAttr(mu)
            api.CreateRestitutionAttr(0.0)
            PhysxSchema.PhysxMaterialAPI.Apply(
                mat.GetPrim()
            ).CreateFrictionCombineModeAttr("min")
            return mat

        steel = material("/World/Steel", 0.3)
        grain_mat = material("/World/GrainMaterial", 0.65)
        # Reference vehicle/floor contact combines to 1.0; grain priority selects .65.
        ground = material("/World/GroundMaterial", 1.0)
        vehicle_mat = material("/World/VehicleMaterial", 1.0)
        UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(
            ground, materialPurpose="physics"
        )
        actor_to_body = {"/World/Floor": 0}
        body_paths = {}
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            # Match complete hierarchy suffixes, not ambiguous bare 'rod' suffixes.
            suffix = str(prim.GetPath()).split("/Geometry/")[-1].replace("/", "_")
            candidates = [
                b
                for b in range(1, m.nbody)
                if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "").endswith(
                    "_" + suffix
                )
            ]
            if len(candidates) != 1:
                raise ValueError("USD body mapping failed: " + suffix)
            body_paths[candidates[0]] = str(prim.GetPath())
            actor_to_body[str(prim.GetPath())] = candidates[0]
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateLinearDampingAttr(0.0)
            api.CreateAngularDampingAttr(0.0)
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            if candidates[0] == rec.bucket:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    steel, materialPurpose="physics"
                )
        # Dynamic material: exact sphere masses and initial free-body poses.
        for b in rec.grains:
            path = "/World/Grains/" + mujoco.mj_id2name(
                m, mujoco.mjtObj.mjOBJ_BODY, int(b)
            )
            geom = int(np.flatnonzero(m.geom_bodyid == b)[0])
            r = float(m.geom_size[geom, 0])
            sphere = UsdGeom.Sphere.Define(stage, path)
            sphere.CreateRadiusAttr(r)
            sphere.AddTranslateOp().Set(Gf.Vec3d(*initial.xpos[b]))
            UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
            UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
            mass = UsdPhysics.MassAPI.Apply(sphere.GetPrim())
            mass.CreateMassAttr(float(m.body_mass[b]))
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*m.body_inertia[b]))
            mass.CreateCenterOfMassAttr(Gf.Vec3f(0))
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(sphere.GetPrim())
            api.CreateLinearDampingAttr(0.0)
            api.CreateAngularDampingAttr(0.0)
            collision = PhysxSchema.PhysxCollisionAPI.Apply(sphere.GetPrim())
            collision.CreateRestOffsetAttr(0.0)
            collision.CreateContactOffsetAttr(0.001)
            PhysxSchema.PhysxContactReportAPI.Apply(
                sphere.GetPrim()
            ).CreateThresholdAttr(0.0)
            UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(
                grain_mat, materialPurpose="physics"
            )
            actor_to_body[path] = int(b)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.CollisionAPI) and str(prim.GetPath()).startswith(
                "/World/Loader/"
            ):
                mat = (
                    steel
                    if str(prim.GetPath()).startswith(body_paths[rec.bucket] + "/")
                    else vehicle_mat
                )
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    mat, materialPurpose="physics"
                )
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                api.CreateEnabledSelfCollisionsAttr(True)
                api.CreateSolverPositionIterationCountAttr(
                    parameters["solver"]["position_iterations"]
                )
                api.CreateSolverVelocityIterationCountAttr(
                    parameters["solver"]["velocity_iterations"]
                )
        for signature in m.exclude_signature:
            a, b = int(signature) >> 16, int(signature) & 65535
            if a in body_paths and b in body_paths:
                UsdPhysics.FilteredPairsAPI.Apply(
                    stage.GetPrimAtPath(body_paths[a])
                ).CreateFilteredPairsRel().AddTarget(body_paths[b])
        if (
            np.any(m.dof_armature)
            or np.any(m.dof_damping)
            or np.any(m.dof_frictionloss)
            or np.any(m.jnt_stiffness)
        ):
            raise ValueError(
                "Adapter only maps the current zero-passive-effort reference"
            )
        mapping["passive_joint_terms"] = (
            "zero armature, spring, damping, friction as compiled reference; no hard joint-velocity clamp"
        )
        mapping["native_explicit_body_exclusions"] = len(m.exclude_signature)
        robot = Articulation(
            ArticulationCfg(
                prim_path="/World/Loader",
                spawn=None,
                actuators={
                    "physical_effort": ImplicitActuatorCfg(
                        joint_names_expr=[".*"],
                        joint_effort_limit=float("inf"),
                        joint_velocity_limit=float("inf"),
                        armature=0.0,
                        friction=0.0,
                        dynamic_friction=0.0,
                        viscous_friction=0.0,
                        stiffness=0.0,
                        damping=0.0,
                    )
                },
            )
        )
        latest = []
        contact_diagnostics = {
            "headers": 0,
            "points": 0,
            "mapped_points": 0,
            "unknown_actor_pairs": [],
        }
        from omni.physx import get_physx_simulation_interface

        collect_contacts = False

        def on_contact(headers, contacts):
            if not collect_contacts:
                return
            contact_diagnostics["headers"] += len(headers)
            contact_diagnostics["points"] += len(contacts)
            for h in headers:
                a = actor_to_body.get(str(PhysicsSchemaTools.intToSdfPath(h.actor0)))
                b = actor_to_body.get(str(PhysicsSchemaTools.intToSdfPath(h.actor1)))
                if a is None or b is None:
                    pair = [
                        str(PhysicsSchemaTools.intToSdfPath(h.actor0)),
                        str(PhysicsSchemaTools.intToSdfPath(h.actor1)),
                    ]
                    if (
                        pair not in contact_diagnostics["unknown_actor_pairs"]
                        and len(contact_diagnostics["unknown_actor_pairs"]) < 5
                    ):
                        contact_diagnostics["unknown_actor_pairs"].append(pair)
                    continue
                for c in contacts[
                    h.contact_data_offset : h.contact_data_offset + h.num_contact_data
                ]:
                    contact_diagnostics["mapped_points"] += 1
                    latest.append((a, b, np.asarray(c.impulse) / m.opt.timestep))

        subscription = get_physx_simulation_interface().subscribe_contact_report_events(
            on_contact
        )
        sim.reset()
        q = np.array([tape["initial_qpos"][m.jnt_qposadr[j]] for j in rec.joint_ids])
        indices = [rec.joint_names.index(n) for n in robot.joint_names]
        root_pose = np.r_[initial.xpos[rec.rear], initial.xquat[rec.rear][[1, 2, 3, 0]]]
        robot.write_root_pose_to_sim(torch.tensor(root_pose[None], dtype=torch.float32))
        robot.write_root_velocity_to_sim(torch.zeros((1, 6)))
        robot.write_joint_state_to_sim(
            torch.tensor(q[indices][None], dtype=torch.float32),
            torch.zeros((1, len(indices))),
        )
        robot.reset()
        sim.forward()  # Initial-state kinematics only; no physics advance.
        robot.update(m.opt.timestep)
        body_indices = {name: i for i, name in enumerate(robot.body_names)}
        from isaaclab_physx.physics import PhysxManager

        view = PhysxManager.get_physics_sim_view()
        grains = (
            view.create_rigid_body_view("/World/Grains/*") if len(rec.grains) else None
        )
        grain_paths = list(grains.prim_paths) if grains is not None else []
        import warp as wp

        grain_pose = np.array(
            [
                np.r_[
                    initial.xpos[actor_to_body[path]],
                    initial.xquat[actor_to_body[path]][[1, 2, 3, 0]],
                ]
                for path in grain_paths
            ],
            dtype=np.float32,
        )
        grain_ids = wp.array(np.arange(len(grain_paths)), dtype=wp.uint32, device="cpu")
        if grains is not None:
            grains.set_transforms(
                wp.array(grain_pose, dtype=wp.float32, device="cpu"), grain_ids
            )
            grains.set_velocities(
                wp.zeros((len(grain_paths), 6), dtype=wp.float32, device="cpu"),
                grain_ids,
            )
        sim.forward()
        robot.update(m.opt.timestep)

        def read_state():
            pos = initial.xpos.copy()
            rot = initial.xmat.reshape(m.nbody, 3, 3).copy()
            full = tape["initial_qpos"].copy()
            poses = robot.data.body_link_pose_w.torch[0].cpu().numpy()
            for b, path in body_paths.items():
                if path.split("/")[-1] not in body_indices:
                    raise RuntimeError("PhysX merged/omitted a required body frame")
                pose = poses[body_indices[path.split("/")[-1]]]
                pos[b] = pose[:3]
                mat = np.zeros(9)
                mujoco.mju_quat2Mat(mat, pose[[6, 3, 4, 5]].astype(float))
                rot[b] = mat.reshape(3, 3)
                if b == rec.rear:
                    adr = int(m.jnt_qposadr[int(m.body_jntadr[b])])
                    full[adr : adr + 3] = pose[:3]
                    full[adr + 3 : adr + 7] = pose[[6, 3, 4, 5]]
            poses = (
                grains.get_transforms().numpy()
                if grains is not None
                else np.empty((0, 7))
            )
            for k, path in enumerate(grain_paths):
                b = actor_to_body[path]
                pos[b] = poses[k, :3]
                mat = np.zeros(9)
                mujoco.mju_quat2Mat(mat, poses[k, [6, 3, 4, 5]].astype(float))
                rot[b] = mat.reshape(3, 3)
                j = int(m.body_jntadr[b])
                adr = int(m.jnt_qposadr[j])
                full[adr : adr + 3] = poses[k, :3]
                full[adr + 3 : adr + 7] = poses[k, [6, 3, 4, 5]]
            actual = robot.data.joint_pos.torch[0].cpu().numpy()
            actual_velocity = robot.data.joint_vel.torch[0].cpu().numpy()
            velocity = np.full(m.nv, np.nan)
            for k, name in enumerate(robot.joint_names):
                index = rec.joint_names.index(name)
                full[rec.addresses[index]] = actual[k]
                velocity[m.jnt_dofadr[rec.joint_ids[index]]] = actual_velocity[k]
            return SimpleNamespace(xpos=pos, xmat=rot, qpos=full, qvel=velocity)

        state = read_state()
        initial_error = float(np.max(np.linalg.norm(state.xpos - initial.xpos, axis=1)))
        mapping["maximum_initial_body_position_error_m"] = initial_error
        errors = np.linalg.norm(state.xpos - initial.xpos, axis=1)
        mapping["initial_body_position_errors_m"] = {
            mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(b)): float(errors[b])
            for b in np.flatnonzero(errors > 1e-5)
        }
        initial_angle_error = np.arccos(
            np.clip(
                (
                    np.einsum(
                        "bij,bij->b", state.xmat, initial.xmat.reshape(m.nbody, 3, 3)
                    )
                    - 1
                )
                / 2,
                -1,
                1,
            )
        )
        mapping["maximum_initial_orientation_error_rad"] = float(
            initial_angle_error.max()
        )
        runtime_inertias = robot.root_view.get_inertias().numpy()[0].reshape(-1, 3, 3)
        inertia_errors = []
        for b, path in body_paths.items():
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, m.body_iquat[b])
            rotation = rotation.reshape(3, 3)
            expected = rotation @ np.diag(m.body_inertia[b]) @ rotation.T
            actual = runtime_inertias[body_indices[path.split("/")[-1]]]
            inertia_errors.append(
                np.linalg.norm(actual - expected) / max(np.linalg.norm(expected), 1e-12)
            )
        mapping["maximum_vehicle_inertia_relative_error"] = float(max(inertia_errors))
        actual_masses = robot.root_view.get_masses().numpy()[0]
        mapping["maximum_vehicle_body_mass_error_kg"] = float(
            max(
                abs(actual_masses[body_indices[path.split("/")[-1]]] - m.body_mass[b])
                for b, path in body_paths.items()
            )
        )
        (output / "mapping.json").write_text(json.dumps(mapping, indent=2) + "\n")
        if (
            initial_error > 1e-4
            or mapping["maximum_initial_orientation_error_rad"] > 1e-3
            or mapping["maximum_vehicle_inertia_relative_error"] > 1e-3
        ):
            raise RuntimeError(
                "Initial-state parity failed; no matched benchmark claimed"
            )
        actuator_joint = [int(m.actuator_trnid[a, 0]) for a in range(m.nu)]
        effort_indices = [
            robot.joint_names.index(
                (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or "").split(
                    "_Physics_"
                )[-1]
            )
            for j in actuator_joint
        ]
        startup_s = time.perf_counter() - started
        start = time.perf_counter()
        physics_s = 0.0
        times = []
        count = (
            manifest["control_steps"]
            if max_ticks is None
            else min(max_ticks, manifest["control_steps"])
        )
        for tick in range(count):
            effort = torch.zeros((1, len(robot.joint_names)))
            effort[0, effort_indices] = torch.tensor(
                tape["effort"][tick], dtype=torch.float32
            )
            robot.set_joint_effort_target(effort)
            robot.write_data_to_sim()
            t = time.perf_counter()
            for substep in range(manifest["substeps"]):
                collect_contacts = (
                    tick % manifest["record_every_ticks"] == 0 or tick == count - 1
                ) and substep == manifest["substeps"] - 1
                latest.clear()
                sim.step(render=False)
            physics_s += time.perf_counter() - t
            robot.update(manifest["control_dt_s"])
            if tick % manifest["record_every_ticks"] == 0 or tick == count - 1:
                state = read_state()
                if not np.isfinite(state.xpos).all():
                    raise RuntimeError("Nonfinite PhysX state")
                rec.sample(
                    (tick + 1) * manifest["control_dt_s"],
                    str(tape["phase"][tick]),
                    state,
                    latest.copy(),
                )
                times.append((tick + 1) * manifest["control_dt_s"])
            if tick % 240 == 0:
                print(f"physx: {tick}/{count} ticks", flush=True)
        wall = time.perf_counter() - start
        duration = count * manifest["control_dt_s"]
        subscription.unsubscribe()
        mapping["contact_diagnostics"] = contact_diagnostics
        mapping["contact_forces_available"] = contact_diagnostics["mapped_points"] > 0
        (output / "mapping.json").write_text(json.dumps(mapping, indent=2) + "\n")
        return rec.save(
            output,
            times,
            "Isaac Lab + PhysX CPU",
            {
                "startup_s": startup_s,
                "instrumented_wall_s": wall,
                "physics_step_wall_s": physics_s,
                "simulated_s": duration,
                "instrumented_real_time_factor": duration / wall,
                "physics_step_real_time_factor": duration / physics_s,
            },
            {
                "completed": count == manifest["control_steps"],
                "mapping": mapping,
                "contact_forces_available": mapping["contact_forces_available"],
                "matched_material_model": False,
                "matched_collision_boundary": False,
                "bundle_sha256": manifest["sha256"],
            },
        )
