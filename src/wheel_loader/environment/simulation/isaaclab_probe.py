"""Isaac Lab 3 / Newton fixed-fixture check of the actual passive USD loop."""

import json
from pathlib import Path

import numpy as np


def articulation_config(p, usd):
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab_newton.sim.schemas import NewtonArticulationCfg

    def drive(names, section):
        return ImplicitActuatorCfg(
            joint_names_expr=names,
            joint_effort_limit=p[section]["max_torque"],
            joint_velocity_limit=p[section]["max_velocity"],
            stiffness=p["solver"]["joint_stiffness"],
            damping=p["solver"]["joint_damping"],
        )

    return ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Loader",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(Path(usd).resolve()),
            articulation_props=[NewtonArticulationCfg(self_collision_enabled=True)],
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, p["vehicle"]["articulation_height"] + 0.5), joint_pos={".*": 0.0}
        ),
        actuators={
            "drive": ImplicitActuatorCfg(
                joint_names_expr=[".*wheel_joint"],
                joint_effort_limit=p["wheels"]["max_torque"],
                joint_velocity_limit=p["wheels"]["max_velocity"],
                stiffness=0.0,
                damping=p["solver"]["wheel_damping"],
            ),
            "steering": drive(["articulation_joint"], "articulation"),
            "boom": drive(["boom_joint"], "boom"),
            "tilt": drive(["tilt_joint"], "linkage"),
            # Infinite force bound avoids a degenerate MuJoCo [0,0] actuator range.
            # Zero K/D and no efforts mean these links remain unactuated.
            "passive": ImplicitActuatorCfg(
                joint_names_expr=["bucket_joint", "rod_joint"],
                joint_effort_limit=float("inf"),
                joint_velocity_limit=p["linkage"]["passive_max_velocity"],
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )


def run_probe(p, usd, output):
    import isaaclab.sim as sim_utils
    import torch
    from isaaclab.app import launch_simulation
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.utils import configclass
    from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

    from wheel_loader.vehicle.kinematics import passive_joint_positions

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        loader = articulation_config(p, usd)

    cfg = sim_utils.SimulationCfg(
        dt=p["solver"]["dt"],
        device="cuda:0",
        visualizer_cfgs=[],
        physics=NewtonCfg(
            solver_cfg=MJWarpSolverCfg(
                solver="cg",
                integrator="implicitfast",
                iterations=200,
                njmax=8192,
                nconmax=2048,
                save_to_mjcf=str(output / "effective_model.xml"),
            )
        ),
    )
    max_gap = 0.0
    max_fk = 0.0
    samples = []

    def point(pose, vector):
        x, y, z, w = pose[3:]
        r = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        return pose[:3] + r @ vector

    with launch_simulation(cfg):
        sim = sim_utils.SimulationContext(cfg)
        try:
            scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=12.0))
            sim.reset()
            robot = scene["loader"]
            names, bodies = robot.joint_names, robot.body_names
            target = torch.zeros((1, len(names)), device=sim.device)
            velocity = torch.zeros_like(target)
            for step in range(round(12 / cfg.dt)):
                t = step * cfg.dt
                target[0, names.index("articulation_joint")] = 0.3 * min(
                    1.0, max(0.0, (t - 2) / 2)
                )
                target[0, names.index("boom_joint")] = 0.5 * min(
                    1.0, max(0.0, (t - 5) / 2)
                )
                target[0, names.index("tilt_joint")] = 0.5 * min(
                    1.0, max(0.0, (t - 8) / 2)
                )
                for name in names:
                    if name.endswith("wheel_joint"):
                        velocity[0, names.index(name)] = 0.6 if t < 4 else 0.0
                robot.actuators.target_command.set_position_index(value=target)
                robot.actuators.target_command.set_velocity_index(value=velocity)
                robot.write_data_to_sim()
                sim.step(render=False)
                scene.update(cfg.dt)
                if step % 12:
                    continue
                q = robot.data.joint_pos.torch[0].cpu().numpy()
                poses = robot.data.body_link_pose_w.torch[0].cpu().numpy()
                if not np.isfinite(q).all() or not np.isfinite(poses).all():
                    raise RuntimeError("Nonfinite Isaac Lab state")
                rod = point(
                    poses[bodies.index("rod")],
                    np.array([p["linkage"]["rod_length"], 0, 0]),
                )
                bx, bz = p["linkage"]["bucket_anchor"]
                anchor = point(poses[bodies.index("bucket")], np.array([bx, 0, bz]))
                max_gap = max(max_gap, float(np.linalg.norm(rod - anchor)))
                boom = float(q[names.index("boom_joint")])
                tilt = float(q[names.index("tilt_joint")])
                # Only tolerate small solver excursions at the initial zero mechanical stop.
                # Record solver stop excursions and fail the final gate without aborting evidence collection.
                expected = passive_joint_positions(
                    p,
                    min(p["boom"]["max_angle"], max(0.0, boom)),
                    min(
                        p["linkage"]["max_angle"], max(p["linkage"]["min_angle"], tilt)
                    ),
                )
                max_fk = max(
                    max_fk,
                    abs(
                        float(q[names.index("bucket_joint")]) - expected["bucket_joint"]
                    ),
                )
                samples.append(q.copy())
            data = np.asarray(samples)
            # Challenge the closure independently of the driven coordinates.
            disturbed = robot.data.joint_pos.torch.clone()
            disturbed[0, names.index("rod_joint")] += 0.2
            robot.write_joint_position_to_sim_index(position=disturbed)
            robot.write_joint_velocity_to_sim_index(
                velocity=torch.zeros_like(disturbed)
            )
            robot.reset()
            for _ in range(round(2 / cfg.dt)):
                robot.actuators.target_command.set_position_index(value=target)
                robot.actuators.target_command.set_velocity_index(value=velocity)
                robot.write_data_to_sim()
                sim.step(render=False)
                scene.update(cfg.dt)
            poses = robot.data.body_link_pose_w.torch[0].cpu().numpy()
            recovered_gap = float(
                np.linalg.norm(
                    point(
                        poses[bodies.index("rod")],
                        np.array([p["linkage"]["rod_length"], 0, 0]),
                    )
                    - point(poses[bodies.index("bucket")], np.array([bx, 0, bz]))
                )
            )
            limits_ok = all(
                data[:, names.index(joint)].min() >= p[section]["min_angle"] - 0.005
                and data[:, names.index(joint)].max() <= p[section]["max_angle"] + 0.005
                for joint, section in [
                    ("articulation_joint", "articulation"),
                    ("boom_joint", "boom"),
                    ("tilt_joint", "linkage"),
                    ("bucket_joint", "bucket"),
                ]
            )
            checks = {
                "joint_limits": bool(limits_ok),
                "loop_closure": max_gap < 0.005,
                "passive_fk": max_fk < 0.03,
                "closure_recovery": recovered_gap < 0.005,
                "articulation": bool(
                    np.ptp(data[:, names.index("articulation_joint")]) > 0.2
                ),
                "boom": bool(np.ptp(data[:, names.index("boom_joint")]) > 0.35),
                "bucket": bool(np.ptp(data[:, names.index("bucket_joint")]) > 0.2),
                "wheels": bool(
                    all(
                        np.ptp(data[:, i]) > 0.5
                        for i, n in enumerate(names)
                        if n.endswith("wheel_joint")
                    )
                ),
            }
            result = {
                "runtime": "Isaac Lab 3 / Newton / MuJoCo Warp",
                "fixture": True,
                "checks": checks,
                "maximum_loop_gap_m": max_gap,
                "maximum_passive_fk_error_rad": max_fk,
                "recovered_gap_m": recovered_gap,
                "joint_names": names,
                "joint_min_rad": data.min(0).tolist(),
                "joint_max_rad": data.max(0).tolist(),
                "physical_accuracy": "not established",
            }
            (output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
            np.savez_compressed(
                output / "motion.npz", positions=data, joint_names=np.asarray(names)
            )
            print(json.dumps(result, indent=2), flush=True)
            if not all(checks.values()):
                raise RuntimeError("Isaac Lab passive linkage checks failed")
            return result
        finally:
            sim_utils.SimulationContext.clear_instance()
