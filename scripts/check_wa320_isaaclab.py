"""Validate fixed WA320 USD using the existing Isaac Lab/Newton integration."""

import argparse
from pathlib import Path

from wheel_loader.environment.simulation.isaaclab_probe import run_probe
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    parser.add_argument(
        "--usd",
        type=Path,
        required=True,
        help="Fixed-base asset with mandatory passive loop",
    )
    parser.add_argument("--output", type=Path, default=Path("outputs/wa320/isaaclab"))
    args = parser.parse_args()
    run_probe(load_parameters(args.config), args.usd, args.output)


if __name__ == "__main__":
    main()
