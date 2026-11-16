"""A coarse rigid-sphere pile settling under Isaac Lab/Newton physics.

Supports a free-pile demo and optional scripted loader probe; neither is calibrated soil.
"""
from pathlib import Path
from importlib import metadata
import json
import math
import time
import xml.etree.ElementTree as ET

import numpy as np

from wheel_loader_rl.config import EnvironmentConfig
from wheel_loader_rl.pile import generate_pile, pile_geometry
from wheel_loader_rl.material_metrics import settling_metrics


def simulate_pile(
    cfg: EnvironmentConfig, output: Path, duration_s: float = 5.0,
    drop_height_m: float = 0.0, fps: int = 25, interactive: bool = False, loader_probe: bool = False,
) -> Path:
    """Run one scene and save recorded particle centers plus a technical summary."""
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError('duration_s must be finite and positive')
    if not math.isfinite(drop_height_m) or drop_height_m < 0:
        raise ValueError('drop_height_m must be finite and nonnegative')
    if type(fps) is not int or fps < 1 or cfg.physics_hz % fps:
        raise ValueError('fps must divide physics_hz exactly')
    if cfg.num_envs != 1:
        raise ValueError('pile demo supports one scene; this is not a batch benchmark')
    if loader_probe:
        from wheel_loader_rl.controllers.scripted_loading import validate_loading_limits
        validate_loading_limits(cfg.action_limits)
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.app import launch_simulation
    from isaaclab.assets import AssetBaseCfg, RigidObjectCfg, RigidObjectCollectionCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.utils import configclass
    from isaaclab_newton.physics import NewtonCfg, MJWarpSolverCfg
    from isaaclab_newton.sim.schemas import NewtonCollisionCfg, MujocoCollisionCfg

    centers = generate_pile(cfg.pile, cfg.seed)
    centers[:, 2] += drop_height_m
    objects = {}
    for index, position in enumerate(centers):
        objects[f'grain_{index:04d}'] = RigidObjectCfg(
            prim_path=f'{{ENV_REGEX_NS}}/Grain_{index:04d}',
            spawn=sim_utils.SphereCfg(
                radius=cfg.pile.radius_m,
                rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
                mass_props=sim_utils.MassCfg(mass=cfg.pile.sphere_mass_kg),
                collision_props=[sim_utils.UsdPhysicsCollisionCfg(collision_enabled=True), NewtonCollisionCfg(), MujocoCollisionCfg(condim=cfg.pile.contact_dimensions)],
                physics_material=sim_utils.NewtonMaterialPropertiesCfg(
                    static_friction=cfg.pile.static_friction, dynamic_friction=cfg.pile.dynamic_friction, rolling_friction=cfg.pile.rolling_friction,
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.72, 0.43, 0.16)),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(float(x) for x in position)),
        )

    @configclass
    class PileSceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path='/World/Ground', spawn=sim_utils.GroundPlaneCfg())
        grains = RigidObjectCollectionCfg(rigid_objects=objects)

    if loader_probe:
        from wheel_loader_rl.simulation.loader_probe import loader_config
        PileSceneCfg.loader = loader_config(output)

    visualizers = []
    if interactive:
        from isaaclab_visualizers.newton import NewtonGLVisualizerCfg
        visualizers.append(NewtonGLVisualizerCfg(window_width=1280, window_height=720))
    sim_cfg = sim_utils.SimulationCfg(
        dt=1 / cfg.physics_hz, device='cuda:0', visualizer_cfgs=visualizers,
        physics=NewtonCfg(solver_cfg=MJWarpSolverCfg(
            njmax=max(4096, 40 * len(centers)), nconmax=max(1024, 8 * len(centers)),
            iterations=200 if loader_probe else 100, solver='cg', cone='pyramidal', integrator='implicitfast',
            save_to_mjcf=str(output / 'effective_model.xml'),
        )),
    )
    output.mkdir(parents=True, exist_ok=True)
    steps = math.ceil(duration_s * cfg.physics_hz)
    stride = cfg.physics_hz // fps
    frames, timestamps = [], []
    loader_poses, loader_joints = [], []
    with launch_simulation(sim_cfg):
        sim = sim_utils.SimulationContext(sim_cfg)
        try:
            scene = InteractiveScene(PileSceneCfg(num_envs=1, env_spacing=6.0))
            sim.reset()
            effective_geoms = [geom for geom in ET.parse(output / 'effective_model.xml').getroot().iter('geom')
                               if 'Grain_' in geom.get('name', '')]
            effective_modes = {int(geom.get('condim', '3')) for geom in effective_geoms}
            if len(effective_geoms) != len(centers) or effective_modes != {cfg.pile.contact_dimensions}:
                raise RuntimeError('Exported solver contact dimensions do not match the pile configuration')
            sim.set_camera_view(eye=(cfg.pile.center_x_m + 2 * cfg.pile.half_length_m,
                                     -2 * cfg.pile.half_width_m, 2.5 * cfg.pile.height_m),
                                target=(cfg.pile.center_x_m, 0.0, cfg.pile.height_m / 2))
            grains = scene['grains']
            if loader_probe:
                from wheel_loader_rl.controllers.scripted_loading import loading_targets
                loader = scene['loader']
                joint_indices = [loader.joint_names.index(name) for name in ('travel', 'lift', 'curl')]
                print('Loader joints:', loader.joint_names, 'bodies:', loader.body_names)
            torch.cuda.synchronize()
            started = time.perf_counter()
            for step in range(steps + 1):
                state = grains.data.object_state_w.torch
                if not torch.isfinite(state).all():
                    raise RuntimeError(f'Nonfinite physics state at step {step}')
                if step % stride == 0:
                    frames.append(grains.data.object_pos_w.torch[0].detach().cpu().numpy().copy())
                    timestamps.append(step / cfg.physics_hz)
                    if loader_probe:
                        # Isaac Lab 3 proxy uses xyzw; recorded domain poses use wxyz.
                        pose = loader.data.body_link_pose_w.torch[0].detach().cpu().numpy().copy()
                        pose[:, 3:] = pose[:, [6, 3, 4, 5]]
                        loader_poses.append(pose)
                        loader_joints.append(loader.data.joint_pos.torch[0].detach().cpu().numpy().copy())
                if step == steps or not sim.is_running():
                    break
                if loader_probe:
                    target = torch.tensor(loading_targets((step + 1) / cfg.physics_hz), device=sim.device, dtype=torch.float32)[None]
                    loader.actuators.target_command.set_position_index(value=target, joint_ids=joint_indices)
                    loader.write_data_to_sim()
                    if not (torch.isfinite(loader.data.joint_pos.torch).all()
                            and torch.isfinite(loader.data.joint_vel.torch).all()
                            and torch.isfinite(loader.data.body_link_pose_w.torch).all()):
                        raise RuntimeError('Nonfinite loader joint state')
                sim.step(render=False)
                scene.update(sim_cfg.dt)
                if interactive:
                    sim.render()
                    # Presentation pacing only; it is not part of the physics step.
                    delay = (step + 1) / cfg.physics_hz - (time.perf_counter() - started)
                    if delay > 0:
                        time.sleep(delay)
            torch.cuda.synchronize()
            elapsed = time.perf_counter() - started
        finally:
            sim_utils.SimulationContext.clear_instance()
    positions = np.asarray(frames, dtype=np.float32)
    if positions[-1, :, 2].min() < cfg.pile.radius_m * 0.5:
        raise RuntimeError('Excessive ground penetration in final pile state')
    reference_xy = positions[0, :, :2].mean(axis=0)
    initial_radial = np.linalg.norm(positions[0, :, :2] - reference_xy, axis=1)
    final_radial = np.linalg.norm(positions[-1, :, :2] - reference_xy, axis=1)
    speed_estimate = (positions[-1] - positions[-2]) * fps if len(positions) > 1 else np.zeros_like(positions[-1])
    trace = output / 'pile_motion.npz'
    extra = {}
    if loader_probe:
        extra = {'loader_body_poses': np.asarray(loader_poses), 'loader_joint_positions': np.asarray(loader_joints),
                 'loader_body_names': np.asarray(loader.body_names), 'loader_joint_names': np.asarray(loader.joint_names)}
    np.savez_compressed(trace, positions_m=positions, time_s=np.asarray(timestamps), radius_m=cfg.pile.radius_m, **extra)
    summary = {
        'demo': 'scripted articulated loader/material feasibility probe' if loader_probe else 'coarse rigid-sphere settling; no loader or bucket-transfer test',
        'seed': cfg.seed, 'particle_count': len(centers), 'physics_steps': step,
        'initial_geometry': pile_geometry(cfg.pile),
        'settling': settling_metrics(positions, timestamps, cfg.pile.radius_m) if len(positions) > 1 else None,
        'sphere_diameter_m': 2 * cfg.pile.radius_m,
        'initial_total_particle_mass_kg': len(centers) * cfg.pile.sphere_mass_kg,
        'final_total_particle_mass_kg': len(centers) * cfg.pile.sphere_mass_kg,
        'mass_note': 'Fixed particle count only; bucket-transfer conservation is not tested.',
        'solver': 'mujoco_warp_cg',
        'physics_hz': cfg.physics_hz, 'recording_fps': fps, 'simulated_duration_s': step / cfg.physics_hz,
        'drop_height_m': drop_height_m, 'all_states_finite': True,
        'contact_dimensions': cfg.pile.contact_dimensions,
        'initial_radius_p90_m': float(np.quantile(initial_radial, 0.9)),
        'final_radius_p90_m': float(np.quantile(final_radial, 0.9)),
        'final_top_height_m': float(positions[-1, :, 2].max() + cfg.pile.radius_m),
        'final_elevated_fraction': float(np.mean(positions[-1, :, 2] > 2 * cfg.pile.radius_m)),
        'final_max_speed_estimate_mps': float(np.linalg.norm(speed_estimate, axis=1).max()),
        'initial_mean_height_m': float(positions[0, :, 2].mean()),
        'final_mean_height_m': float(positions[-1, :, 2].mean()),
        'final_min_center_height_m': float(positions[-1, :, 2].min()),
        'wall_time_s': elapsed,
        'versions': {name: metadata.version(name) for name in ('isaaclab', 'newton', 'warp-lang', 'torch')},
        'assumptions': {'static_friction': cfg.pile.static_friction, 'dynamic_friction': cfg.pile.dynamic_friction, 'rolling_friction': cfg.pile.rolling_friction,
                        'sphere_radius_m': cfg.pile.radius_m, 'sphere_mass_kg': cfg.pile.sphere_mass_kg},
    }
    if loader_probe:
        from wheel_loader_rl.material_accounting import transfer_metrics
        summary['material_transfer'] = transfer_metrics(positions, timestamps, np.asarray(loader_poses)[:, loader.body_names.index('bucket')],
                                                        cfg.pile.radius_m, cfg.pile.sphere_mass_kg)
        summary['mass_note'] = 'See material_transfer for identity-based acquisition, retention, release and final partition.'
        summary['settling'] = None  # Moving bucket invalidates the free-pile settling diagnostic.
        summary['loader'] = {'approximation': 'rail chassis, fixed wheels, lift/curl joints; no traction or steering',
                             'joint_names': loader.joint_names, 'body_names': loader.body_names,
                             'recorded_pose_quaternion_order': 'wxyz',
                             'configured_target_rate_limits': {'travel_mps': cfg.action_limits.longitudinal_mps, 'lift_radps': cfg.action_limits.lift_radps, 'curl_radps': cfg.action_limits.curl_radps},
                             'solver_iteration_limit': 200}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return trace
