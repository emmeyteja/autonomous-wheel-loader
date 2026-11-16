"""Regenerate the original parameterized WA320-like URDF and closure sidecar."""

import argparse
from pathlib import Path

from wheel_loader.vehicle.asset import build_asset
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("assets/wheel_loader/wa320")
    )
    args = parser.parse_args()
    print(build_asset(load_parameters(args.config), args.output))


if __name__ == "__main__":
    main()
