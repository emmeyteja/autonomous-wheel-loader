"""Create a video from recorded Newton ground poses, using MuJoCo and ffmpeg."""

import argparse
from pathlib import Path

from wheel_loader.environment.simulation.replay_video import render_replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", type=Path, required=True, help="Ground probe output folder"
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--speedup", type=float, default=1.0)
    args = parser.parse_args()
    print(render_replay(args.input, args.output, fps=args.fps, speedup=args.speedup))


if __name__ == "__main__":
    main()
