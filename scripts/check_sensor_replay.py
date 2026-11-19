"""Generate offline sensor streams from an accepted recorded rigid-pile maneuver."""

import argparse
import json
from pathlib import Path

import yaml

from wheel_loader.environment.simulation.sensor_replay import run_sensor_replay
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("outputs/wa320/sensors"))
    parser.add_argument(
        "--config", type=Path, default=Path("configs/sensors/diagnostic.yaml")
    )
    parser.add_argument(
        "--vehicle", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    args = parser.parse_args()
    result = run_sensor_replay(
        args.input,
        args.output,
        load_parameters(args.vehicle),
        yaml.safe_load(args.config.read_text()),
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
