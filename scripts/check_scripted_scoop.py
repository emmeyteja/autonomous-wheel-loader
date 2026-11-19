"""Run a small deterministic material-scoop experiment; no learning."""

import argparse
from pathlib import Path

import yaml

from wheel_loader.environment.simulation.scoop_probe import run_scoop
from wheel_loader.vehicle.parameters import load_parameters

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/experiments/scripted_scoop.yaml")
    )
    parser.add_argument("--output", type=Path, default=Path("outputs/wa320/m2/nominal"))
    parser.add_argument("--scenario", default="nominal")
    args = parser.parse_args()
    result = run_scoop(
        load_parameters("configs/vehicle/wa320.yaml"),
        yaml.safe_load(args.config.read_text()),
        args.source,
        args.output,
        args.scenario,
    )
    if not result["single_run_passed"]:
        raise SystemExit("M2 acceptance failed; see report.json")
