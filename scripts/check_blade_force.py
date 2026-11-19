"""Run a bounded, explicitly uncalibrated blade-force diagnostic."""

import argparse
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

from wheel_loader.environment.simulation.blade_probe import run_blade


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/experiments/blade_force.yaml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("outputs/blade_force/nominal")
    )
    parser.add_argument("--width", type=float, default=0.1)
    parser.add_argument("--depth", type=float, default=0.04)
    parser.add_argument("--material", choices=["gravel", "dry_sand"])
    parser.add_argument("--empty", action="store_true")
    parser.add_argument("--half-dt", action="store_true")
    parser.add_argument("--finer", action="store_true")
    parser.add_argument("--matrix", action="store_true")
    parser.add_argument(
        "--jobs",
        type=int,
        default=1,
        choices=[1, 2, 3],
        help="Bounded independent matrix processes",
    )
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    if args.material is not None:
        config["material"] = args.material
    if args.half_dt:
        config["dt_s"] /= 2
    if args.finer:
        config["radius_m"] *= 0.8
    if args.matrix:
        (args.output / "matrix.json").unlink(missing_ok=True)

        def execute_case(case):
            width, depth = case
            name = f"w{round(width * 1000)}_d{round(depth * 1000)}"
            directory = args.output / name
            (directory / "validation.json").unlink(missing_ok=True)
            command = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--config",
                str(args.config),
                "--width",
                str(width),
                "--depth",
                str(depth),
                "--output",
                str(directory),
            ]
            if args.material is not None:
                command.extend(["--material", args.material])
            if args.half_dt:
                command.append("--half-dt")
            if args.finer:
                command.append("--finer")
            if args.empty:
                command.append("--empty")
            process = subprocess.run(command, check=False)
            report = directory / "validation.json"
            return name, {
                "process_passed": process.returncode == 0,
                "validation": json.loads(report.read_text())
                if report.exists()
                else None,
            }

        cases = [
            (width, depth)
            for width in config["blade_widths_m"]
            for depth in config["cutting_depths_m"]
        ]
        with ThreadPoolExecutor(max_workers=args.jobs) as workers:
            results = dict(workers.map(execute_case, cases))
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "matrix.json").write_text(json.dumps(results, indent=2) + "\n")
        if not all(r["process_passed"] for r in results.values()):
            raise SystemExit("Blade matrix failed; inspect matrix.json")
        return
    result = run_blade(config, args.width, args.depth, args.output, empty=args.empty)
    print(json.dumps(result, indent=2))
    if not all(result["checks"].values()):
        raise SystemExit("Blade numerical diagnostic failed")


if __name__ == "__main__":
    main()
