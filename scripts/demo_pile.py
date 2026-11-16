"""Run a real pile-motion demo and optionally export a video from recorded state."""
import argparse
from dataclasses import replace
from pathlib import Path

from wheel_loader_rl.config import load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('configs/pile_trapezoid_dense.toml'))
    parser.add_argument('--seed', type=int)
    parser.add_argument('--duration-s', type=float, default=5.0)
    parser.add_argument('--drop-height-m', type=float, default=0.0)
    parser.add_argument('--fps', type=int, default=25)
    parser.add_argument('--interactive', action='store_true', help='Open the Newton viewer; requires a desktop display.')
    parser.add_argument('--no-video', action='store_true', help='Save physics data only; skip ffmpeg rendering.')
    parser.add_argument('--output', type=Path, default=Path('outputs/pile_demo'))
    args = parser.parse_args()
    cfg = load_config(args.config)
    if args.seed is not None:
        cfg = replace(cfg, seed=args.seed)
    from wheel_loader_rl.simulation.pile_demo import simulate_pile
    trace = simulate_pile(cfg, args.output, args.duration_s, args.drop_height_m, args.fps, args.interactive)
    if not args.no_video:
        from wheel_loader_rl.visualization.pile_video import render_pile_video
        print(render_pile_video(trace, args.output / 'pile_settling.mp4', args.fps))


if __name__ == '__main__':
    main()
