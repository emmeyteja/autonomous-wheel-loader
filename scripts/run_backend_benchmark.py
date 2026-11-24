"""Replay one immutable physical-command bundle; no training or backend tuning."""

import argparse
import json
from pathlib import Path

from wheel_loader.environment.simulation.backend_benchmark import run_replay
from wheel_loader.vehicle.parameters import load_parameters

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--bundle", type=Path, default=Path("outputs/wa320/backend_benchmark/bundle")
    )
    p.add_argument("--backend", choices=["native", "mjwarp"], required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument(
        "--max-ticks",
        type=int,
        help="Smoke-test only; incomplete, never a full comparison",
    )
    a = p.parse_args()
    result = run_replay(
        a.bundle,
        a.output,
        a.backend,
        load_parameters("configs/vehicle/wa320.yaml"),
        a.max_ticks,
    )
    print(json.dumps(result, indent=2))
    if not result["completed"]:
        raise SystemExit("Incomplete benchmark")
