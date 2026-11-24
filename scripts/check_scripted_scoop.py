"""Run a deterministic coarse-material scoop experiment; no learning."""

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
    parser.add_argument(
        "--output", type=Path, help="Default: outputs/wa320/m2/<scenario>"
    )
    parser.add_argument(
        "--scenario", help="Default is the configured research scenario"
    )
    parser.add_argument(
        "--wall-budget-s",
        type=float,
        help="Explicit incomplete/fail CPU budget, including compile",
    )
    parser.add_argument("--load-response", choices=("legacy", "stop_and_curl"))
    parser.add_argument("--reaction-feedback-hz", type=float)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    if args.wall_budget_s is not None:
        if args.wall_budget_s <= 0:
            parser.error("--wall-budget-s must be positive")
        config["wall_budget_s"] = args.wall_budget_s
    if args.load_response is not None:
        config["controller"]["load_response"] = args.load_response
    if args.reaction_feedback_hz is not None:
        config["reaction_feedback_hz"] = args.reaction_feedback_hz
    scenario = args.scenario or config.get("default_scenario", "nominal")
    result = run_scoop(
        load_parameters("configs/vehicle/wa320.yaml"),
        config,
        args.source,
        args.output or Path("outputs/wa320/m2") / scenario,
        scenario,
    )
    if not result["single_run_passed"]:
        raise SystemExit("M2 acceptance failed; see report.json")
