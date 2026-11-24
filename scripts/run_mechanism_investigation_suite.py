"""Small reproducible investigation matrix; outputs/logs stay local and ignored."""

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

from wheel_loader.environment.simulation.backend_benchmark import load_bundle

SCRIPT = Path(__file__).with_name("investigate_mechanism_sensitivity.py")


def runtime_warnings(lines):
    return [
        line
        for line in lines
        if not line.lstrip().startswith('"')
        and re.search(
            r"\bwarning\b|\boverflow\b|iterations limit reached|non.?finite",
            line,
            re.IGNORECASE,
        )
    ]


def invoke(arguments, output=None):
    env = {**os.environ, "PYTHONPATH": "src"}
    command = [sys.executable, str(SCRIPT), *map(str, arguments)]
    if output is None:
        subprocess.run(command, env=env, check=True)
    else:
        output.mkdir(parents=True, exist_ok=True)
        with (output / "run.log").open("w") as log:
            subprocess.run(
                command, env=env, stdout=log, stderr=subprocess.STDOUT, check=True
            )
        lines = (output / "run.log").read_text().splitlines()
        warnings = runtime_warnings(lines)
        (output / "runtime_warning_lines.json").write_text(
            json.dumps(warnings, indent=2) + "\n"
        )


def prepare(folder, case, mode="baseline", solver="CG", seconds=10, passive=False):
    bundle = folder / "bundle"
    if bundle.exists():
        model, _, m = load_bundle(bundle)
        inv = m["investigation"]
        expected_solver = 1 if solver == "CG" else 2
        if (
            inv["perturbation"] != mode
            or inv.get("zero_effort_ablation", False) != passive
            or m["scenario"] != case
            or abs(m["duration_s"] - seconds) > 1e-8
            or int(model.opt.solver) != expected_solver
        ):
            raise ValueError("Existing bundle differs from requested experiment")
    else:
        invoke(
            [
                "prepare",
                "--case",
                case,
                "--perturbation",
                mode,
                "--solver",
                solver,
                "--seconds",
                seconds,
                "--output",
                bundle,
                *(["--zero-effort"] if passive else []),
            ]
        )
    return bundle


def run(bundle, backend, output, reference=None):
    if (output / "report.json").exists():
        report = json.loads((output / "report.json").read_text())
        _, _, manifest = load_bundle(bundle)
        if (
            not report["completed"]
            or report["bundle_sha256"] != manifest["sha256"]
            or report.get("capacity_check_scope")
            != "every GPU physics substep and observation forward pass"
        ):
            raise ValueError(
                "Existing run is incomplete or mismatched; use a fresh output root"
            )
        return
    invoke(
        [
            "run",
            "--bundle",
            bundle,
            "--backend",
            backend,
            "--output",
            output,
            *(["--feedback-reference", reference] if reference else []),
        ],
        output,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--root", type=Path, default=Path("outputs/wa320/sensitivity_investigation")
    )
    p.add_argument(
        "--phase",
        choices=["sensitivity", "full-newton", "feedback", "all"],
        default="all",
    )
    a = p.parse_args()
    root = a.root
    if a.phase in ["sensitivity", "all"]:
        for name, case in [
            ("fixed", "fixed_no_contact"),
            ("ground", "unloaded_ground"),
            ("pile", "pile_reference"),
        ]:
            for mode in ["baseline", "isolated", "consistent"]:
                folder = root / name / mode
                bundle = prepare(folder, case, mode)
                for rep in range(1, 4 if mode == "baseline" else 2):
                    run(bundle, "native", folder / f"native_{rep}")
            for suffix, solver, passive in [
                ("newton", "Newton", False),
                ("passive", "CG", True),
            ]:
                if name == "pile" and passive:
                    continue
                for mode in ["baseline", "consistent"]:
                    folder = root / (name + "_" + suffix) / mode
                    bundle = prepare(folder, case, mode, solver, passive=passive)
                    run(bundle, "native", folder / "native_1")
        for mode in ["baseline", "consistent"]:
            folder = root / "floating_no_contact" / mode
            bundle = prepare(folder, "floating_no_contact", mode)
            run(bundle, "native", folder / "native_1")
    if a.phase in ["full-newton", "all"]:
        folder = root / "full_newton"
        bundle = prepare(
            folder, "pile_reference", solver="Newton", seconds=33.30833333333333
        )
        for backend in ["native", "mjwarp"]:
            for rep in [1, 2]:
                run(bundle, backend, folder / f"{backend}_{rep}")
    if a.phase in ["feedback", "all"]:
        folder = root / "ground_newton/baseline"
        bundle = prepare(folder, "unloaded_ground", solver="Newton")
        run(bundle, "native", folder / "native_1")
        reference = folder / "native_1/states.npz"
        for backend in ["native", "mjwarp"]:
            run(bundle, backend, root / "feedback_ground" / backend, reference)
