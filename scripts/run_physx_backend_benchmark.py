"""Run the PhysX adapter from the existing full Isaac Sim environment."""

import argparse
import json
import sys
from pathlib import Path

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--bundle", type=Path, default=Path("outputs/wa320/backend_benchmark/bundle")
    )
    p.add_argument(
        "--usd",
        type=Path,
        required=True,
        help="Derived v1 USD used to export the reference model",
    )
    p.add_argument(
        "--lab-site-packages",
        type=Path,
        required=True,
        help="Existing Isaac Lab site-packages; no package upgrades",
    )
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--max-ticks", type=int)
    a = p.parse_args()
    sys.path.append(str(a.lab_site_packages.resolve()))
    from wheel_loader.environment.simulation.physx_backend_benchmark import run_physx
    from wheel_loader.vehicle.parameters import load_parameters

    a.output.mkdir(parents=True, exist_ok=True)
    (a.output / "blocked.json").unlink(missing_ok=True)
    try:
        result = run_physx(
            a.bundle,
            a.usd,
            a.output,
            load_parameters("configs/vehicle/wa320.yaml"),
            a.max_ticks,
        )
    except Exception as exc:
        (a.output / "blocked.json").write_text(
            json.dumps(
                {
                    "backend": "Isaac Lab + PhysX",
                    "status": "BLOCKED",
                    "exception_type": type(exc).__name__,
                    "reason": str(exc),
                    "metrics_available": False,
                },
                indent=2,
            )
            + "\n"
        )
        raise
    print(json.dumps(result, indent=2))
