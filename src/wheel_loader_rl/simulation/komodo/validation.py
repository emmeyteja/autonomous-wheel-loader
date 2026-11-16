"""Single-scene articulation checks; not a loading task or machine-accuracy test."""
from pathlib import Path
import json
import math
import numpy as np
from .inspection import MOVING_JOINTS, forward_kinematics


def articulation_config(usd_path):
    import isaaclab.sim as sim_utils
    from isaaclab.assets import ArticulationCfg
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_newton.sim.schemas import NewtonArticulationCfg
    return ArticulationCfg(
        prim_path='{ENV_REGEX_NS}/Komodo',
        spawn=sim_utils.UsdFileCfg(usd_path=str(Path(usd_path).resolve()), variants={'Physics': 'mujoco'},
            articulation_props=[NewtonArticulationCfg(self_collision_enabled=True)]),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0, 0, .4), joint_pos={'.*': 0.0}),
        actuators={
            'wheels': ImplicitActuatorCfg(joint_names_expr=['.*wheel_joint'], joint_effort_limit=.05,
                                          joint_velocity_limit=32., stiffness=0., damping=.2),
            'arm': ImplicitActuatorCfg(joint_names_expr=['arm_joint'], joint_effort_limit=100.,
                                       joint_velocity_limit=.2, stiffness=1000., damping=10.),
            'bucket': ImplicitActuatorCfg(joint_names_expr=['bucket_joint'], joint_effort_limit=30.,
                                          joint_velocity_limit=.01, stiffness=70., damping=.01),
        })


def transform_xyzw(pose):
    x, y, z, w = pose[3:]
    result = np.eye(4)
    result[:3, 3] = pose[:3]
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    return result


def validate_articulation(usd_path, mapping_path, output, duration_s=24.):
    """Exercise wheel direction, lift, curl and relative link FK in a fixed-base fixture."""
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.app import launch_simulation
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.utils import configclass
    from isaaclab_newton.physics import NewtonCfg, MJWarpSolverCfg
    if duration_s < 24 or not math.isfinite(duration_s):
        raise ValueError('At least 24 seconds required for the joint check sequence')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    mapping = json.loads(Path(mapping_path).read_text())

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path='/World/Ground', spawn=sim_utils.GroundPlaneCfg())
        loader = articulation_config(usd_path)

    cfg = sim_utils.SimulationCfg(dt=.005, device='cuda:0', visualizer_cfgs=[],
        physics=NewtonCfg(solver_cfg=MJWarpSolverCfg(njmax=4096, nconmax=1024, iterations=200,
            solver='cg', integrator='implicitfast', save_to_mjcf=str(output/'effective_model.xml'))))
    positions, velocities, poses, timestamps = [], [], [], []
    max_fk_position_error, max_fk_rotation_error = 0., 0.
    with launch_simulation(cfg):
        sim = sim_utils.SimulationContext(cfg)
        try:
            scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=3))
            sim.reset()
            loader = scene['loader']
            names, bodies = loader.joint_names, loader.body_names
            if set(names) != set(MOVING_JOINTS):
                raise RuntimeError(f'Unexpected converted joints: {names}')
            wheels = [i for i, name in enumerate(names) if name.endswith('wheel_joint')]
            arm, bucket = names.index('arm_joint'), names.index('bucket_joint')
            q_target = torch.zeros((1, len(names)), device=sim.device)
            v_target = torch.zeros_like(q_target)
            for step in range(round(duration_s/cfg.dt)+1):
                t = step*cfg.dt
                q, qd = loader.data.joint_pos.torch, loader.data.joint_vel.torch
                body_poses = loader.data.body_link_pose_w.torch
                if not (torch.isfinite(q).all() and torch.isfinite(qd).all() and torch.isfinite(body_poses).all()):
                    raise RuntimeError('Nonfinite Komodo articulation state')
                if step % 20 == 0:
                    positions.append(q[0].cpu().numpy().copy())
                    velocities.append(qd[0].cpu().numpy().copy())
                    pose = body_poses[0].cpu().numpy().copy()
                    poses.append(pose)
                    timestamps.append(t)
                    expected = forward_kinematics(mapping['joints'], dict(zip(names, positions[-1])))
                    root_pose = transform_xyzw(pose[bodies.index('base_footprint')])
                    for index, name in enumerate(bodies):
                        actual = np.linalg.inv(root_pose) @ transform_xyzw(pose[index])
                        max_fk_position_error = max(max_fk_position_error, float(np.linalg.norm(actual[:3, 3]-expected[name][:3, 3])))
                        max_fk_rotation_error = max(max_fk_rotation_error, float(np.linalg.norm(actual[:3, :3]-expected[name][:3, :3])))
                if step == round(duration_s/cfg.dt):
                    break
                v_target.zero_()
                for i in wheels:
                    if 2 <= t < 5:
                        v_target[0, i] = 1.
                    elif 5 <= t < 8:
                        v_target[0, i] = 1. if 'left' in names[i] else -1.
                q_target[0, arm] = min(.2, max(0., (t-8)*.05))
                q_target[0, bucket] = min(.05, max(0., (t-14)*.01))
                loader.actuators.target_command.set_position_index(value=q_target)
                loader.actuators.target_command.set_velocity_index(value=v_target)
                loader.write_data_to_sim()
                sim.step(render=False)
                scene.update(cfg.dt)
            q = np.asarray(positions)
            qd = np.asarray(velocities)
            index = lambda seconds: int(np.argmin(abs(np.asarray(timestamps)-seconds)))
            spin = q[index(5), wheels]-q[index(2), wheels]
            differential = qd[index(7), wheels]
            expected_sign = np.array([1 if 'left' in names[i] else -1 for i in wheels])
            checks = {
                'wheel_rotation': bool(np.all(spin > .2)),
                'differential_wheel_direction': bool(np.all(differential*expected_sign > .1)),
                'lift_arm_motion': bool(np.ptp(q[:, arm]) > .08),
                'bucket_curl': bool(np.ptp(q[:, bucket]) > .015),
                'joint_limits_during_motion': bool(q[:, arm].min() >= -.102 and q[:, arm].max() <= .322
                                                  and q[:, bucket].min() >= -.502 and q[:, bucket].max() <= .902),
                'urdf_relative_link_frames': bool(max_fk_position_error < 1e-4 and max_fk_rotation_error < 1e-4),
            }
            stops = []
            for name, limit, sign in (('arm_joint', .32, 1), ('arm_joint', -.1, -1),
                                       ('bucket_joint', .9, 1), ('bucket_joint', -.5, -1)):
                joint_index = names.index(name)
                near = torch.zeros_like(q_target)
                near[0, joint_index] = limit-sign*.002
                loader.write_joint_position_to_sim_index(position=near)
                loader.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(near))
                loader.reset()
                q_target.copy_(near)
                q_target[0, joint_index] = limit+sign*.15
                v_target.zero_()
                observed = []
                for _ in range(400):
                    loader.actuators.target_command.set_position_index(value=q_target)
                    loader.actuators.target_command.set_velocity_index(value=v_target)
                    loader.write_data_to_sim()
                    sim.step(render=False)
                    scene.update(cfg.dt)
                    value = float(loader.data.joint_pos.torch[0, joint_index])
                    if not math.isfinite(value):
                        raise RuntimeError('Nonfinite joint stop challenge')
                    observed.append(value)
                excess = max(0., max(sign*(value-limit) for value in observed))
                stops.append({'joint': name, 'limit_rad': limit, 'target_rad': limit+sign*.15,
                              'maximum_excess_rad': excess, 'final_distance_to_stop_rad': abs(observed[-1]-limit),
                              'passed': bool(excess <= .005 and abs(observed[-1]-limit) <= .005)})
            checks['joint_stop_challenges'] = all(stop['passed'] for stop in stops)
            from .collision_checks import check_collision_filters
            collision = check_collision_filters(output/'effective_model.xml')
            checks['self_collision_positive_control'] = collision['artificial_nonadjacent_overlap_detected']
            np.savez_compressed(output/'articulation_motion.npz', joint_positions=q, joint_velocities=qd,
                                body_poses_xyzw=np.asarray(poses), time_s=timestamps,
                                joint_names=np.asarray(names), body_names=np.asarray(bodies))
            result = {'variant': 'Komodo diagnostic with primitive attachment proxies and repaired wheel inertias',
                      'runtime': 'Isaac Lab 3 / Newton MuJoCo Warp', 'checks': checks,
                      'max_fk_position_error_m': max_fk_position_error, 'max_fk_rotation_matrix_error': max_fk_rotation_error,
                      'joint_names': names, 'body_names': bodies,
                      'joint_min_rad': q.min(0).tolist(), 'joint_max_rad': q.max(0).tolist(),
                      'chassis_steering_articulation': 'absent in upstream; unsupported',
                      'ground_skid_steering': 'not tested in suspended fixed-base fixture',
                      'self_collision': collision,
                      'joint_hard_stop_challenge': stops,
                      'physical_accuracy': 'not established'}
            (output/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result, indent=2))
            return result
        finally:
            sim_utils.SimulationContext.clear_instance()
