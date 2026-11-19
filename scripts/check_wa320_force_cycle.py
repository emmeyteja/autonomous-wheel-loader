"""Run the complete normalized force-driven cycle as a native CPU diagnostic."""

import argparse
import json
from pathlib import Path

from wheel_loader.environment.simulation.reconstruction_probe import run_v_cycle_probe
from wheel_loader.vehicle.parameters import load_parameters

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument(
        "--output", type=Path, default=Path("outputs/wa320/reconstructed_v1/cpu_cycle")
    )
    args = parser.parse_args()
    r = run_v_cycle_probe(
        load_parameters("configs/vehicle/wa320.yaml"), args.source, args.output
    )
    print(json.dumps(r, indent=2))
    if not all(r["checks"].values()):
        raise SystemExit("Full force-driven cycle failed")
