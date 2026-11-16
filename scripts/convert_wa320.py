"""Convert the original WA320-like tree and passive closure using Isaac Sim."""

import argparse
from pathlib import Path

from wheel_loader.environment.simulation.conversion import convert
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    parser.add_argument(
        "--urdf", type=Path, default=Path("assets/wheel_loader/wa320/wa320.urdf")
    )
    parser.add_argument("--output", type=Path, default=Path("outputs/wa320/usd"))
    parser.add_argument("--fixed", action="store_true")
    args = parser.parse_args()
    print(
        convert(load_parameters(args.config), args.urdf, args.output, fixed=args.fixed)
    )


if __name__ == "__main__":
    main()
