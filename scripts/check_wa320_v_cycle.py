"""Run the full V-cycle motion rehearsal on Newton rigid ground."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

from wheel_loader.control.v_cycle import VCycleScenario
from wheel_loader.environment.simulation.ground_contact import load_ground_profile
from wheel_loader.environment.simulation.newton_ground import run_ground
from wheel_loader.environment.simulation.newton_profile import load_profile
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--case", choices=["all", "nominal", "slow", "wide"], default="all"
    )
    parser.add_argument("--output", type=Path, default=Path("outputs/wa320/v_cycle"))
    args = parser.parse_args()
    if args.case == "all":
        results = {}
        for name in ("nominal", "slow", "wide"):
            directory = args.output / name
            result = subprocess.run(
                [
                    sys.executable,
                    str(Path(__file__).resolve()),
                    "--case",
                    name,
                    "--output",
                    str(directory),
                ],
                check=False,
            )
            record = directory / "validation.json"
            results[name] = {
                "process_passed": result.returncode == 0,
                "validation": json.loads(record.read_text())
                if record.exists()
                else None,
            }
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "suite.json").write_text(json.dumps(results, indent=2) + "\n")
        if not all(item["process_passed"] for item in results.values()):
            raise SystemExit("V-cycle suite failed; inspect suite.json")
        return
    run_ground(
        load_parameters("configs/vehicle/wa320.yaml"),
        Path("assets/wheel_loader/wa320/usd/wa320.usda"),
        args.output,
        profile=load_profile("configs/simulation/newton.yaml"),
        contact_profile=load_ground_profile("configs/simulation/newton_ground.yaml"),
        scenario=VCycleScenario.named(args.case),
    )


if __name__ == "__main__":
    main()
