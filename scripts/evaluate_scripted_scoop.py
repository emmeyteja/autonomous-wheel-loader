"""Aggregate three M2 scenarios and an identical-condition repeat; no simulation."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def evaluate(folder, cfg):
    reports = {
        name: json.loads((folder / name / "report.json").read_text())
        for name in (*cfg["scenarios"], "nominal_repeat")
    }
    first, repeat = reports["nominal"], reports["nominal_repeat"]
    with (
        np.load(folder / "nominal/recording.npz") as a,
        np.load(folder / "nominal_repeat/recording.npz") as b,
    ):
        same_times = np.array_equal(a["time_s"], b["time_s"])
        position_error = (
            float(np.max(abs(a["qpos"] - b["qpos"])))
            if a["qpos"].shape == b["qpos"].shape
            else None
        )
    expected_digest = hashlib.sha256(
        json.dumps(cfg, sort_keys=True).encode()
    ).hexdigest()
    checks = {
        "reports_match_requested_configuration": all(
            r["experiment_sha256"] == expected_digest for r in reports.values()
        ),
        "same_source_model": len({r["source_sha256"] for r in reports.values()}) == 1,
        "repeat_sample_times": same_times,
        "measurable_bucket_spillage_in_suite": max(
            r["peak_measured_bucket_exit_mass_kg"] for r in reports.values()
        )
        >= cfg["acceptance"]["spilled_mass_min_kg"],
        "all_scenario_physics": all(r["single_run_passed"] for r in reports.values()),
        "same_prepared_initial_state": first["prepared_state_sha256"]
        == repeat["prepared_state_sha256"],
        "same_settled_initial_state": first["initial_state_sha256"]
        == repeat["initial_state_sha256"],
        "same_vehicle": len({r["parameter_sha256"] for r in reports.values()}) == 1,
        "same_experiment": len({r["experiment_sha256"] for r in reports.values()}) == 1,
        "repeat_state_error": position_error is not None
        and position_error <= cfg["acceptance"]["repeat_position_tolerance_m"],
        "repeat_payload_error": abs(
            first["retained_payload_kg"] - repeat["retained_payload_kg"]
        )
        <= cfg["acceptance"]["repeat_mass_tolerance_kg"],
    }
    result = {
        "checks": checks,
        "m2_cpu_reference_passed": all(checks.values()),
        "isaaclab_newton_gpu_material_execution_verified": False,
        "calibrated_material": False,
        "maximum_repeat_generalized_position_error": position_error,
        "interpretation": "CPU rigid-grain reference only; angular and linear generalized positions use one strict numerical repeat screen, not physical accuracy",
        "scenarios": reports,
    }
    (folder / "suite.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/experiments/scripted_scoop.yaml")
    )
    args = parser.parse_args()
    result = evaluate(args.folder, yaml.safe_load(args.config.read_text()))
    print(json.dumps(result["checks"], indent=2))
    if not result["m2_cpu_reference_passed"]:
        raise SystemExit("M2 CPU reference acceptance failed")
