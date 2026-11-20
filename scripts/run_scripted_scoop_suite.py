"""Sequential CPU reservoir study. A failed physical gate is retained, not skipped."""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument(
        "--output", type=Path, default=Path("outputs/wa320/m2/reservoir_suite")
    )
    parser.add_argument("--benchmark-wall-s", type=float, default=120)
    parser.add_argument("--research-wall-s", type=float, default=900)
    parser.add_argument(
        "--reuse-existing",
        action="store_true",
        help="Reuse reports; evaluator still checks physical configuration",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    jobs = [
        "small_debug",
        "nominal",
        "nominal_repeat",
        "research_underburden",
        "research_underburden_repeat",
        "low_shallow",
        "low_shallow_wide",
        "tall_deep",
        "spill_demonstration",
        "debug_1500",
        "medium",
        "large",
    ]
    statuses = {}
    env = dict(
        os.environ, PYTHONPATH="src", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1"
    )
    for name in jobs:
        dest = args.output / name
        if args.reuse_existing and (dest / "report.json").exists():
            statuses[name] = "reused; configuration checked by evaluator"
            continue
        scenario = name.removesuffix("_repeat")
        budget = (
            args.benchmark_wall_s
            if name in ("debug_1500", "medium", "large")
            else args.research_wall_s
        )
        command = [
            sys.executable,
            "scripts/check_scripted_scoop.py",
            "--source",
            str(args.source),
            "--scenario",
            scenario,
            "--output",
            str(dest),
            "--wall-budget-s",
            str(budget),
        ]
        print(f"Running {name}, CPU wall budget {budget}s", flush=True)
        with (args.output / (name + ".log")).open("w") as log:
            try:
                status = subprocess.run(
                    command,
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=budget + 120,
                    check=False,
                ).returncode
                statuses[name] = {
                    "exit_code": status,
                    "report_written": (dest / "report.json").exists(),
                }
            except subprocess.TimeoutExpired:
                statuses[name] = {
                    "external_timeout": True,
                    "report_written": (dest / "report.json").exists(),
                    "interpretation": "compile/individual solver step/report exceeded budget; not a completed scoop",
                }
        (args.output / "execution_status.json").write_text(
            json.dumps(statuses, indent=2) + "\n"
        )
    result = subprocess.run(
        [sys.executable, "scripts/evaluate_scripted_scoop.py", str(args.output)],
        env=env,
        check=False,
    )
    raise SystemExit(result.returncode)
