"""Resolve the active vehicle without importing optional references or simulators."""

import argparse
import json
from pathlib import Path

import yaml

from wheel_loader.evaluation.foundation import check_geometry
from wheel_loader.vehicle.registry import select_vehicle


def main(argv=None):
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(
        description="WA320 foundation checks; learning is not implemented."
    )
    parser.add_argument(
        "--config", type=Path, default=root / "configs/training/default.yaml"
    )
    parser.add_argument("--vehicle", choices=["wa320", "komodo"])
    parser.add_argument("--mode", choices=["foundation", "ppo"])
    parser.add_argument("--allow-reference", action="store_true")
    args = parser.parse_args(argv)
    cfg = yaml.safe_load(args.config.read_text())
    name = args.vehicle or cfg["vehicle_model"]
    try:
        model = select_vehicle(name, root=root, allow_reference=args.allow_reference)
    except ValueError as error:
        parser.error(str(error))
    if name == "komodo":
        print(
            json.dumps(
                {"reference": model.path, "active": False, "training_available": False}
            )
        )
        return
    if (args.mode or cfg["mode"]) != "foundation":
        parser.error(
            "PPO is unavailable: validate dynamics, terrain and scripted scooping first (M1–M4)."
        )
    if cfg.get("terrain") != "rigid_pile":
        parser.error(
            "Only the rigid-pile foundation is available; granular backends are not calibrated."
        )
    print(json.dumps(check_geometry(model.parameters), indent=2))
