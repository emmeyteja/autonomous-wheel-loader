"""Render an accepted numerical column diagnostic from saved positions."""

import argparse
from pathlib import Path

from wheel_loader.environment.simulation.column_video import render_column


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    print(render_column(args.input, args.output))


if __name__ == "__main__":
    main()
