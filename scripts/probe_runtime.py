"""Seeded finite-state GPU probe of UPSTREAM cartpole, not the loader task."""
import argparse
from importlib import metadata
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--num-envs', type=int, default=1)
    parser.add_argument('--steps', type=int, default=100)
    parser.add_argument('--output', type=Path, default=Path('outputs/m0/runtime_probe.json'))
    args = parser.parse_args()
    if args.num_envs <= 0 or args.steps <= 0:
        parser.error('num-envs and steps must be positive')
    import torch
    from isaaclab.app import launch_simulation
    from isaaclab_newton.physics import NewtonCfg, MJWarpSolverCfg
    from isaaclab_tasks.core.cartpole.cartpole_direct_env_cfg import CartpoleEnvCfg
    from isaaclab_tasks.core.cartpole.cartpole_direct_env import CartpoleEnv
    cfg = CartpoleEnvCfg()
    cfg.seed = args.seed
    cfg.scene.num_envs = args.num_envs
    cfg.sim.device = 'cuda:0'
    cfg.sim.physics = NewtonCfg(solver_cfg=MJWarpSolverCfg(njmax=5, nconmax=3, cone='pyramidal', impratio=1, integrator='implicitfast'))
    cfg.sim.default_visualizer_cfg = None
    cfg.sim.visualizer_cfgs = []
    with launch_simulation(cfg):
        env = CartpoleEnv(cfg)
        try:
            observations, _ = env.reset(seed=args.seed)
            initial = observations['policy'].clone()
            torch.cuda.synchronize()
            start = time.perf_counter()
            for _ in range(args.steps):
                actions = torch.zeros((args.num_envs, 1), device=env.device)
                observations, rewards, terminated, truncated, _ = env.step(actions)
                if not torch.isfinite(observations['policy']).all() or not torch.isfinite(rewards).all():
                    raise RuntimeError('upstream runtime produced nonfinite output')
            torch.cuda.synchronize()
            elapsed = time.perf_counter() - start
            observations, _ = env.reset(seed=args.seed)
            reset_matches = torch.allclose(observations['policy'], initial, rtol=0, atol=1e-6)
            reset_error = (observations['policy'] - initial).abs().max().item()
            report = {
                'status': 'PASS' if reset_matches else 'FAIL_RESET_CHECK', 'task': 'UPSTREAM Isaac-Cartpole-Direct; not wheel loader',
                'seed': args.seed, 'num_envs': args.num_envs, 'steps': args.steps,
                'finite_observations_rewards': True, 'seeded_reset_matches': reset_matches, 'reset_tolerance': 1e-6,
                'reset_max_absolute_error': reset_error,
                'initial_observation': initial.cpu().tolist(),
                'reset_observation': observations['policy'].cpu().tolist(),
                'elapsed_s': elapsed, 'environment_steps_per_s': args.steps * args.num_envs / elapsed,
                'torch_peak_allocated_bytes': torch.cuda.max_memory_allocated(),
                'versions': {name: metadata.version(name) for name in ('isaaclab','torch','newton','warp-lang')},
            }
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
            if not reset_matches:
                raise RuntimeError('Upstream cartpole reset observation mismatch; see runtime_probe.json. This is not a loader task result.')
        finally:
            env.close()


if __name__ == '__main__':
    main()
