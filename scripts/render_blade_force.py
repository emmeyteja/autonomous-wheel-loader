"""Render the recorded states and contact forces from an accepted diagnostic."""

import argparse
from pathlib import Path

from wheel_loader.environment.simulation.blade_video import render_blade

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--input", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
if __name__ == "__main__":
    args = parser.parse_args()
    print(render_blade(args.input, args.output))
