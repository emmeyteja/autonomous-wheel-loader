"""Run A-G native CPU mass-distribution diagnostics; no learning or hardware."""

import argparse
import json
from pathlib import Path

from wheel_loader.environment.simulation.reconstruction_probe import run_probes
from wheel_loader.vehicle.parameters import load_parameters

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument(
        "--output", type=Path, default=Path("outputs/wa320/reconstructed_v1/dynamics")
    )
    args = parser.parse_args()
    result = run_probes(
        load_parameters("configs/vehicle/wa320.yaml"), args.source, args.output
    )
    print(json.dumps(result, indent=2))
    if not all(all(r["checks"].values()) for r in result.values()):
        raise SystemExit("Dynamic diagnostic failed; inspect suite.json")
