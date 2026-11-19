"""Render accepted recorded motion beside its generated sensor map."""

import argparse
from pathlib import Path

from wheel_loader.environment.simulation.sensor_video import render_sensor_video
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--sensors", type=Path, default=Path("outputs/wa320/sensors"))
    parser.add_argument(
        "--vehicle", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    args = parser.parse_args()
    print(render_sensor_video(args.input, args.sensors, load_parameters(args.vehicle)))


if __name__ == "__main__":
    main()
