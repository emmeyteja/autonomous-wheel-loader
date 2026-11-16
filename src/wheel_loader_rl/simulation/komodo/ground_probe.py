"""Ground skid-steering feasibility probe using source wheel effort limits."""
from pathlib import Path
import json
import numpy as np
from .validation import articulation_config, transform_xyzw


def probe_ground(usd, output):
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.app import launch_simulation
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.utils import configclass
    from isaaclab_newton.physics import NewtonCfg, MJWarpSolverCfg
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    robot_cfg = articulation_config(usd)
    robot_cfg.init_state.pos = (0, 0, .005)

    @configclass
    class GroundScene(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path='/World/Ground', spawn=sim_utils.GroundPlaneCfg())
        loader = robot_cfg

    cfg = sim_utils.SimulationCfg(dt=.005, device='cuda:0', visualizer_cfgs=[],
        physics=NewtonCfg(solver_cfg=MJWarpSolverCfg(njmax=4096, nconmax=1024, iterations=200,
            solver='cg', integrator='implicitfast', save_to_mjcf=str(output/'effective_model.xml'))))
    roots, speeds, times = [], [], []
    with launch_simulation(cfg):
        sim = sim_utils.SimulationContext(cfg)
        try:
            scene = InteractiveScene(GroundScene(num_envs=1, env_spacing=3))
            sim.reset()
            robot = scene['loader']
            names = robot.joint_names
            wheels = [i for i, name in enumerate(names) if name.endswith('wheel_joint')]
            target = torch.zeros((1, len(names)), device=sim.device)
            velocity = torch.zeros_like(target)
            for step in range(3001):
                t = step*cfg.dt
                q, v = robot.data.joint_pos.torch, robot.data.joint_vel.torch
                pose = robot.data.body_link_pose_w.torch
                if not (torch.isfinite(q).all() and torch.isfinite(v).all() and torch.isfinite(pose).all()):
                    raise RuntimeError('Nonfinite ground motion')
                if step % 20 == 0:
                    roots.append(pose[0, robot.body_names.index('base_footprint')].cpu().numpy().copy())
                    speeds.append(v[0].cpu().numpy().copy())
                    times.append(t)
                if step == 3000:
                    break
                velocity.zero_()
                if 2 <= t < 12:
                    for i in wheels:
                        velocity[0, i] = 8. if t < 7 or 'left' in names[i] else -8.
                robot.actuators.target_command.set_position_index(value=target)
                robot.actuators.target_command.set_velocity_index(value=velocity)
                robot.write_data_to_sim()
                sim.step(render=False)
                scene.update(cfg.dt)
            roots = np.asarray(roots)
            yaw = np.unwrap([np.arctan2(transform_xyzw(p)[1, 0], transform_xyzw(p)[0, 0]) for p in roots])
            at = lambda t: int(np.argmin(abs(np.asarray(times)-t)))
            turn = float(yaw[at(12)]-yaw[at(7)])
            travel = float(np.linalg.norm(roots[at(7), :2]-roots[at(2), :2]))
            result = {'runtime': 'Isaac Lab 3 / Newton', 'variant': 'free-base Komodo diagnostic',
                      'wheel_effort_limit_Nm': .05, 'wheel_target_radps': 8.,
                      'straight_phase_displacement_m': travel, 'turn_phase_yaw_change_rad': turn,
                      'straight_screen_passed': travel > .1, 'turn_screen_passed': abs(turn) > .1,
                      'chassis_steering_hinge': 'absent; this is skid steering',
                      'all_states_finite': True, 'contact_parameters_calibrated': False,
                      'warning': 'Project screening thresholds .1m and .1rad; no tire traction validation'}
            np.savez_compressed(output/'ground_motion.npz', body_pose_xyzw=roots, time_s=times,
                                joint_velocities=np.asarray(speeds), joint_names=np.asarray(names))
            (output/'ground_probe.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result, indent=2))
            return result
        finally:
            sim_utils.SimulationContext.clear_instance()
