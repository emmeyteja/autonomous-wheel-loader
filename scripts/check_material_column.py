"""Run isolated material-contact diagnostics; no field-calibration claim."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import yaml

from wheel_loader.environment.simulation.column_probe import run_column


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/experiments/material_column.yaml")
    )
    parser.add_argument("--material", choices=["gravel", "dry_sand"])
    parser.add_argument("--output", type=Path, default=Path("outputs/material_column"))
    parser.add_argument("--suite", action="store_true")
    parser.add_argument(
        "--case",
        choices=["nominal", "repeat_seed", "half_dt", "finer_elements"],
        default="nominal",
    )
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    material = args.material or config["default_material"]
    cases = {
        "nominal": {},
        "repeat_seed": {"seed": 43},
        "half_dt": {"dt": config["dt_s"] / 2},
        "finer_elements": {"radius": 0.01, "dt": config["dt_s"] / 2},
    }
    if args.suite:
        results = {}
        for name in cases:
            directory = args.output / name
            (directory / "validation.json").unlink(missing_ok=True)
            process = subprocess.run(
                [
                    sys.executable,
                    str(Path(__file__).resolve()),
                    "--config",
                    str(args.config),
                    "--material",
                    material,
                    "--case",
                    name,
                    "--output",
                    str(directory),
                ],
                check=False,
            )
            path = directory / "validation.json"
            results[name] = {
                "process_passed": process.returncode == 0,
                "validation": json.loads(path.read_text()) if path.exists() else None,
            }
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "suite.json").write_text(json.dumps(results, indent=2) + "\n")
        if not all(result["process_passed"] for result in results.values()):
            raise SystemExit("Numerical column suite failed; inspect suite.json")
        return
    result = run_column(config, material, args.output, **cases[args.case])
    print(json.dumps(result, indent=2), flush=True)
    if not all(result["checks"].values()):
        raise SystemExit("Numerical column screen failed; inspect validation.json")


if __name__ == "__main__":
    main()
