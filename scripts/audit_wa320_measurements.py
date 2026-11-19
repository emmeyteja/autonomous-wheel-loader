"""Export every active parameter with its source, units and unknown true error."""

import argparse
import json
from pathlib import Path

import yaml

from wheel_loader.evaluation.measurement_audit import audit_parameters
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--vehicle", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    parser.add_argument(
        "--ledger", type=Path, default=Path("configs/measurements/wa320_7.yaml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("outputs/wa320/measurements.json")
    )
    args = parser.parse_args()
    result = audit_parameters(
        load_parameters(args.vehicle), yaml.safe_load(args.ledger.read_text())
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"Classified {len(result['active'])} active parameters; true vehicle errors remain unknown."
    )


if __name__ == "__main__":
    main()
