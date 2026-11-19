"""Additional Newton fixture diagnostics; no training policy or hardware commands."""

import numpy as np

from wheel_loader.vehicle.kinematics import passive_joint_positions


def transform_point(pose, vector):
    x, y, z, w = pose[3:]
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    return pose[:3] + rotation @ np.asarray(vector)


def linkage_gap(robot, p, env_index=0):
    poses = robot.data.body_link_pose_w.torch[env_index].cpu().numpy()
    rod = transform_point(
        poses[robot.body_names.index("rod")], [p["linkage"]["rod_length"], 0, 0]
    )
    bx, bz = p["linkage"]["bucket_anchor"]
    bucket = transform_point(poses[robot.body_names.index("bucket")], [bx, 0, bz])
    gaps = [float(np.linalg.norm(rod - bucket))]
    for name, spec in p.get("cylinder_bodies", {}).items():
        tip = transform_point(
            poses[robot.body_names.index(spec["tip_body"])], spec["tip"]
        )
        endpoint = poses[robot.body_names.index(name + "_rod"), :3]
        gaps.append(float(np.linalg.norm(tip - endpoint)))
    return max(gaps)


def set_pose(robot, sim, p, *, boom=0.6, tilt=0.0, articulation=0.0):
    import torch

    positions = passive_joint_positions(p, boom, tilt, articulation=articulation) | {
        "articulation_joint": articulation
    }
    target = torch.zeros((1, len(robot.joint_names)), device=sim.device)
    for name, value in positions.items():
        target[0, robot.joint_names.index(name)] = value
    robot.write_joint_position_to_sim_index(position=target)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(target))
    robot.reset()
    return target


def advance(robot, scene, sim, target, dt):
    import torch

    robot.actuators.target_command.set_position_index(value=target)
    robot.actuators.target_command.set_velocity_index(value=torch.zeros_like(target))
    robot.write_data_to_sim()
    sim.step(render=False)
    scene.update(dt)


def joint_stop_challenges(robot, scene, sim, p, dt, sampled_positions=None):
    results = []
    for joint, section in [
        ("articulation_joint", "articulation"),
        ("boom_joint", "boom"),
        ("tilt_joint", "linkage"),
    ]:
        for bound, sign in [("min_angle", -1), ("max_angle", 1)]:
            limit = p[section][bound]
            near = limit - sign * 0.002
            pose_args = {
                "articulation_joint": "articulation",
                "boom_joint": "boom",
                "tilt_joint": "tilt",
            }
            target = set_pose(robot, sim, p, **{pose_args[joint]: near})
            target[0, robot.joint_names.index(joint)] = limit + sign * 0.15
            max_excess = 0.0
            max_gap = 0.0
            value = float("nan")
            for _ in range(round(2 / dt)):
                advance(robot, scene, sim, target, dt)
                if sampled_positions is not None:
                    sampled_positions.append(
                        robot.data.joint_pos.torch[0].cpu().numpy().copy()
                    )
                value = float(
                    robot.data.joint_pos.torch[0, robot.joint_names.index(joint)]
                )
                gap = linkage_gap(robot, p)
                if not np.isfinite(value) or not np.isfinite(gap):
                    raise RuntimeError("Nonfinite joint-stop challenge")
                max_excess = max(max_excess, sign * (value - limit))
                max_gap = max(max_gap, gap)
            results.append(
                {
                    "joint": joint,
                    "limit_rad": limit,
                    "target_rad": limit + sign * 0.15,
                    "maximum_excess_rad": max_excess,
                    "final_distance_rad": abs(value - limit),
                    "maximum_loop_gap_m": max_gap,
                    "passed": bool(
                        max_excess <= 0.005
                        and abs(value - limit) <= 0.005
                        and max_gap < 0.005
                    ),
                }
            )
    return results


def reset_repeatability(robot, scene, sim, p, dt):
    trajectories = []
    max_gap = 0.0
    for _ in range(2):
        target = set_pose(robot, sim, p, boom=0.3, tilt=-0.25, articulation=0.2)
        values = []
        for _ in range(round(2 / dt)):
            advance(robot, scene, sim, target, dt)
            q = robot.data.joint_pos.torch[0].cpu().numpy().copy()
            if not np.isfinite(q).all():
                raise RuntimeError("Nonfinite reset trajectory")
            values.append(q)
            max_gap = max(max_gap, linkage_gap(robot, p))
        trajectories.append(np.asarray(values))
    error = float(np.max(np.abs(trajectories[1] - trajectories[0])))
    return {
        "maximum_joint_replay_error_rad": error,
        "maximum_loop_gap_m": max_gap,
        "passed": bool(error < 1e-4 and max_gap < 0.005),
    }


def self_collision_sweep(path, positions, joint_names):
    """Evaluate recorded joint states with the exported MuJoCo collision geometry.

    Includes an artificial nonadjacent overlap to catch disabled self-collision.
    This is a collision-geometry check, not a second dynamics simulation.
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    joint_ids = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i).split("_Physics_")[-1]: i
        for i in range(model.njnt)
    }
    addresses = [model.jnt_qposadr[joint_ids[name]] for name in joint_names]
    minimum = 0.0
    pairs = set()
    for q in positions:
        data.qpos[addresses] = q
        mujoco.mj_forward(model, data)
        for contact in data.contact[: data.ncon]:
            if contact.dist < minimum:
                minimum = float(contact.dist)
            if contact.dist < -0.002:
                names = [
                    mujoco.mj_id2name(
                        model,
                        mujoco.mjtObj.mjOBJ_BODY,
                        int(model.geom_bodyid[int(geom)]),
                    ).split("_Geometry_")[-1]
                    for geom in contact.geom
                ]
                pairs.add(" | ".join(sorted(names)))
    # Prove that collision filters do not suppress a deliberate remote-body overlap.
    body_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
        for i in range(model.nbody)
    ]
    rear = next(i for i, name in enumerate(body_names) if name.endswith("_rear_frame"))
    rod = next(
        i for i, name in enumerate(body_names) if name.endswith("_bellcrank_rod")
    )
    rear_geom = next(i for i in range(model.ngeom) if model.geom_bodyid[i] == rear)
    rod_geom = next(i for i in range(model.ngeom) if model.geom_bodyid[i] == rod)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    displaced_center = data.xmat[rod].reshape(3, 3).T @ (
        data.geom_xpos[rear_geom] - data.xpos[rod]
    )
    # Geometry edits require rebuilding MuJoCo's compiled body/BVH bounds.
    # Editing geom_pos in-place alone can leave broadphase bounds stale.
    spec = mujoco.MjSpec.from_file(str(path))
    spec.geom(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, rod_geom)
    ).pos = displaced_center
    overlap_model = spec.compile()
    overlap_data = mujoco.MjData(overlap_model)
    mujoco.mj_forward(overlap_model, overlap_data)
    filters_unchanged = bool(
        np.array_equal(overlap_model.geom_contype, model.geom_contype)
        and np.array_equal(overlap_model.geom_conaffinity, model.geom_conaffinity)
    )
    detected = any(
        set(map(int, contact.geom)) == {rear_geom, rod_geom} and contact.dist < -0.002
        for contact in overlap_data.contact[: overlap_data.ncon]
    )
    return {
        "sample_count": len(positions),
        "minimum_separation_m": minimum,
        "penetrating_body_pairs": sorted(pairs),
        "artificial_overlap_detected": detected,
        "positive_control_filters_unchanged": filters_unchanged,
        "passed": bool(not pairs and detected and filters_unchanged),
    }


def ground_reset_check(robot, scene, sim, p, dt):
    """Reset the floating loader twice and compare identical stationary replays."""
    import torch

    target = torch.zeros((1, len(robot.joint_names)), device=sim.device)
    for name, value in passive_joint_positions(
        p, *p.get("maneuver", {}).get("settle", [0.15, -0.3])
    ).items():
        target[0, robot.joint_names.index(name)] = value
    root = torch.tensor(
        [[0.0, 0, p["vehicle"]["articulation_height"] + 0.02, 0, 0, 0, 1]],
        device=sim.device,
    )
    runs = []
    gap = 0.0
    reset_error = 0.0
    for _ in range(2):
        robot.write_joint_position_to_sim_index(position=target)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(target))
        robot.write_root_pose_to_sim_index(root_pose=root)
        robot.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros((1, 6), device=sim.device)
        )
        robot.reset()
        scene["wheel_contacts"].reset()
        reset_error = max(
            reset_error,
            float(torch.max(torch.abs(robot.data.joint_pos.torch - target))),
        )
        trajectory = []
        if "actuators" in p:
            from wheel_loader.control.commands import LoaderCommand
            from wheel_loader.control.cylinder_actuation import (
                FirstOrderCylinders,
                joint_efforts,
                normalized_reference,
            )

            actuator = FirstOrderCylinders(p, [0, *p["maneuver"]["settle"]])
            reference = LoaderCommand(
                boom_angle=p["maneuver"]["settle"][0],
                tilt_angle=p["maneuver"]["settle"][1],
            )
        for _ in range(round(2 / dt)):
            if "actuators" in p:
                q = robot.data.joint_pos.torch[0].cpu().numpy()
                qd = robot.data.joint_vel.torch[0].cpu().numpy()
                angles = [
                    float(q[robot.joint_names.index(n)])
                    for n in ("articulation_joint", "boom_joint", "tilt_joint")
                ]
                u = normalized_reference(p, reference, angles)
                force, _ = joint_efforts(p, actuator, u, q, qd, robot.joint_names, dt)
                robot.actuators.target_command.set_effort_index(
                    value=torch.as_tensor(
                        force[None, :], dtype=torch.float32, device=sim.device
                    )
                )
                robot.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)
            else:
                advance(robot, scene, sim, target, dt)
            q = robot.data.joint_pos.torch[0].cpu().numpy()
            pose = robot.data.root_link_pose_w.torch[0].cpu().numpy()
            trajectory.append(np.concatenate([q, pose]))
            gap = max(gap, linkage_gap(robot, p))
        runs.append(np.asarray(trajectory))
    error = float(np.max(np.abs(runs[1] - runs[0])))
    return {
        "maximum_joint_reset_error_rad": reset_error,
        "maximum_replay_component_error": error,
        "maximum_loop_gap_m": gap,
        "passed": bool(
            np.isfinite(runs).all()
            and reset_error < 1e-6
            and error < 1e-4
            and gap < 0.005
        ),
    }


def recorded_cylinder_checks(path, p, recording):
    """Independent FK audit of recorded states; no physics stepping or correction."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    names = recording["joint_names"].tolist()
    q = np.asarray(recording["positions"])
    joints = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j).split("_Physics_")[-1]: j
        for j in range(model.njnt)
    }
    addresses = [model.jnt_qposadr[joints[n]] for n in names]
    body_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i): i
        for i in range(model.nbody)
    }
    from wheel_loader.vehicle.mass_properties import body_path_suffix

    ids = {
        n: next(
            i for full, i in body_names.items() if full.endswith(body_path_suffix(p, n))
        )
        for n in p["bodies"]
    }
    maximum = {n: 0.0 for n in p.get("cylinder_bodies", {})}
    stroke_overruns = {}
    for row in q:
        data.qpos[addresses] = row
        mujoco.mj_kinematics(model, data)
        for name, spec in p.get("cylinder_bodies", {}).items():
            tip = ids[spec["tip_body"]]
            end = ids[name + "_rod"]
            gap = float(
                np.linalg.norm(
                    data.xpos[end]
                    - data.xpos[tip]
                    - data.xmat[tip].reshape(3, 3) @ spec["tip"]
                )
            )
            maximum[name] = max(maximum[name], gap)
    for name, spec in p.get("cylinder_bodies", {}).items():
        c = p["cylinders"][spec["kind"]]
        x = (
            q[:, names.index(name + "_slider_joint")]
            + spec["reference_length"]
            - c["pin_distance_min"]
        )
        stroke_overruns[name] = float(
            np.maximum(np.maximum(-x, x - c["stroke"]), 0).max()
        )
    return {
        "checks": {
            "physical_cylinder_pin_closure": all(g < 0.001 for g in maximum.values()),
            "all_physical_cylinder_strokes": all(
                g < 0.001 for g in stroke_overruns.values()
            ),
        },
        "samples": len(q),
        "maximum_pin_gap_m": maximum,
        "maximum_stroke_overrun_m": stroke_overruns,
        "method": "FK of every recorded state, not physics replay or state correction",
    }
