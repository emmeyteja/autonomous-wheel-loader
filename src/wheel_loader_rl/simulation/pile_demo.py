"""A coarse rigid-sphere pile settling under Isaac Lab/Newton physics.

This is a material-motion demo, not calibrated soil or a bucket-transfer test.
"""
from pathlib import Path
from importlib import metadata
import json
import math
import time

import numpy as np

from wheel_loader_rl.config import EnvironmentConfig
from wheel_loader_rl.pile import generate_pile


def simulate_pile(
    cfg: EnvironmentConfig, output: Path, duration_s: float = 5.0,
    drop_height_m: float = 0.8, fps: int = 25, interactive: bool = False,
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
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.app import launch_simulation
    from isaaclab.assets import AssetBaseCfg, RigidObjectCfg, RigidObjectCollectionCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.utils import configclass
    from isaaclab_newton.physics import NewtonCfg, MJWarpSolverCfg
    from isaaclab_newton.sim.schemas import NewtonCollisionCfg

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
                collision_props=[sim_utils.UsdPhysicsCollisionCfg(collision_enabled=True), NewtonCollisionCfg()],
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

    visualizers = []
    if interactive:
        from isaaclab_visualizers.newton import NewtonGLVisualizerCfg
        visualizers.append(NewtonGLVisualizerCfg(window_width=1280, window_height=720))
    sim_cfg = sim_utils.SimulationCfg(
        dt=1 / cfg.physics_hz, device='cuda:0', visualizer_cfgs=visualizers,
        physics=NewtonCfg(solver_cfg=MJWarpSolverCfg(
            njmax=2048, nconmax=1024, cone='pyramidal', integrator='implicitfast',
        )),
    )
    output.mkdir(parents=True, exist_ok=True)
    steps = math.ceil(duration_s * cfg.physics_hz)
    stride = cfg.physics_hz // fps
    frames, timestamps = [], []
    with launch_simulation(sim_cfg):
        sim = sim_utils.SimulationContext(sim_cfg)
        try:
            scene = InteractiveScene(PileSceneCfg(num_envs=1, env_spacing=6.0))
            sim.reset()
            sim.set_camera_view(eye=(5.0, -4.0, 3.0), target=(cfg.pile.center_x_m, 0.0, 0.4))
            grains = scene['grains']
            torch.cuda.synchronize()
            started = time.perf_counter()
            for step in range(steps + 1):
                state = grains.data.object_state_w.torch
                if not torch.isfinite(state).all():
                    raise RuntimeError(f'Nonfinite physics state at step {step}')
                if step % stride == 0:
                    frames.append(grains.data.object_pos_w.torch[0].detach().cpu().numpy().copy())
                    timestamps.append(step / cfg.physics_hz)
                if step == steps or not sim.is_running():
                    break
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
    trace = output / 'pile_motion.npz'
    np.savez_compressed(trace, positions_m=positions, time_s=np.asarray(timestamps), radius_m=cfg.pile.radius_m)
    summary = {
        'demo': 'coarse rigid-sphere settling; no loader or bucket-transfer test',
        'seed': cfg.seed, 'particle_count': len(centers), 'physics_steps': step,
        'physics_hz': cfg.physics_hz, 'recording_fps': fps, 'simulated_duration_s': step / cfg.physics_hz,
        'drop_height_m': drop_height_m, 'all_states_finite': True,
        'initial_mean_height_m': float(positions[0, :, 2].mean()),
        'final_mean_height_m': float(positions[-1, :, 2].mean()),
        'final_min_center_height_m': float(positions[-1, :, 2].min()),
        'wall_time_s': elapsed,
        'versions': {name: metadata.version(name) for name in ('isaaclab', 'newton', 'warp-lang', 'torch')},
        'assumptions': {'static_friction': cfg.pile.static_friction, 'dynamic_friction': cfg.pile.dynamic_friction, 'rolling_friction': cfg.pile.rolling_friction,
                        'sphere_radius_m': cfg.pile.radius_m, 'sphere_mass_kg': cfg.pile.sphere_mass_kg},
    }
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return trace
