"""Capture one unchanged M2 physical-effort tape; outputs remain local."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from wheel_loader.environment.simulation.scoop_probe import run_scoop
from wheel_loader.vehicle.parameters import load_parameters

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument(
        "--output", type=Path, default=Path("outputs/wa320/backend_benchmark/bundle")
    )
    args = ap.parse_args()
    cfg = yaml.safe_load(Path("configs/experiments/scripted_scoop.yaml").read_text())
    cfg["capture_commands"] = True
    p = load_parameters("configs/vehicle/wa320.yaml")
    result = run_scoop(p, cfg, args.source, args.output, "small_debug")
    if not result["single_run_passed"]:
        raise SystemExit("Reference scoop failed; benchmark bundle not approved")
    with np.load(args.output / "command_tape.npz") as n:
        steps = len(n["effort"])
        dt = float(n["control_dt_s"])
    manifest = {
        "schema_version": 1,
        "scenario": "small_debug",
        "vehicle_commit": "97ae4e5",
        "parameter_sha256": result["parameter_sha256"],
        "control_steps": steps,
        "control_dt_s": dt,
        "duration_s": steps * dt,
        "substeps": 8,
        "record_every_ticks": 12,
        "comparison": "sim-to-sim; no real WA320 accuracy ground truth",
        "actuation": "identical open-loop physical motor effort tape; no per-backend feedback controller",
        "sha256": {
            name: hashlib.sha256((args.output / name).read_bytes()).hexdigest()
            for name in ["compiled_model.mjb", "model.xml", "command_tape.npz"]
        },
    }
    (args.output / "benchmark.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))
