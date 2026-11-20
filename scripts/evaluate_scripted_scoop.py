"""Aggregate M2 reservoir sensitivity, physical exits and unchanged shallow gates."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml


def controlled_factors(report):
    return {
        "controller": report["controller"],
        "material": report["material_parameters"],
        "bucket_contact": report["bucket_contact_parameters"],
        "radius": report["pile_preparation"]["radius_m"],
        "seed": report["pile_preparation"]["seed"],
        "solver": report["native_contact_solver"],
        "source": report["source_sha256"],
        "vehicle": report["parameter_sha256"],
    }


def matches_case(report, config, scenario):
    case = dict(config["scenarios"][scenario])
    controller = dict(config["controller"])
    for key in (
        "attack_angle_deg",
        "attack_lip_height_m",
        "penetration_depth_m",
        "demonstrate_spillage",
    ):
        if key in case:
            controller[key] = case.pop(key)
    material = dict(config["material"])
    material["particle_density_kg_m3"] = case.pop(
        "density_kg_m3", material["particle_density_kg_m3"]
    )
    budget = case.pop("particle_budget", config["max_particles"])
    return (
        report["scenario"] == scenario
        and report["controller"] == controller
        and report["material_parameters"] == material
        and report["bucket_contact_parameters"] == config["bucket_contact"]
        and report["pile_preparation"]["max_particles"] == budget
        and report["pile_preparation"]["radius_m"] == config["radius_m"]
        and report["pile_preparation"]["seed"] == config["seed"]
        and all(report["pile_preparation"].get(k) == v for k, v in case.items())
    )


def evaluate(folder, cfg):
    required = [
        "small_debug",
        "nominal",
        "medium",
        "debug_1500",
        "large",
        "low_shallow",
        "low_shallow_wide",
        "spill_demonstration",
        "tall_deep",
        "nominal_repeat",
        "research_underburden",
        "research_underburden_repeat",
    ]
    reports = {
        name: json.loads((folder / name / "report.json").read_text())
        for name in required
        if (folder / name / "report.json").exists()
    }
    missing = [name for name in required if name not in reports]
    first, repeat = reports.get("nominal"), reports.get("nominal_repeat")
    same_times, position_error = False, None
    if first and repeat:
        with (
            np.load(folder / "nominal/recording.npz") as a,
            np.load(folder / "nominal_repeat/recording.npz") as b,
        ):
            same_times = np.array_equal(a["time_s"], b["time_s"])
            if a["qpos"].shape == b["qpos"].shape:
                position_error = float(np.max(abs(a["qpos"] - b["qpos"])))
    sensitivity = [
        reports[n] for n in ("small_debug", "medium", "large") if n in reports
    ]
    checks = {
        "required_reports_present": not missing,
        "reports_match_requested_physics": bool(reports)
        and all(
            matches_case(r, cfg, n.removesuffix("_repeat")) for n, r in reports.items()
        ),
        "same_source_model": len({r["source_sha256"] for r in reports.values()}) == 1,
        "same_vehicle": len({r["parameter_sha256"] for r in reports.values()}) == 1,
        "identical_factors_for_pile_size_sensitivity": len(sensitivity) == 3
        and all(
            controlled_factors(r) == controlled_factors(sensitivity[0])
            for r in sensitivity
        ),
        "all_requested_scoops_completed": not missing
        and all(r["checks"]["sequence_completed"] for r in reports.values()),
        "measurable_physical_mouth_spillage_in_suite": any(
            r["acquisition_funnel"]["spilled_during_extraction_unique"]["mass_kg"]
            >= cfg["acceptance"]["spilled_mass_min_kg"]
            for r in reports.values()
        ),
        "all_scenario_physics": not missing
        and all(r["single_run_passed"] for r in reports.values()),
        "low_shallow_acquisition_passed": bool(
            reports.get("low_shallow", {})
            .get("checks", {})
            .get("positive_retained_payload", False)
        ),
        "repeat_sample_times": same_times,
        "same_prepared_initial_state": bool(
            first
            and repeat
            and first["prepared_state_sha256"] == repeat["prepared_state_sha256"]
        ),
        "same_settled_initial_state": bool(
            first
            and repeat
            and first["initial_state_sha256"] is not None
            and first["initial_state_sha256"] == repeat["initial_state_sha256"]
        ),
        "repeat_state_error": position_error is not None
        and position_error <= cfg["acceptance"]["repeat_position_tolerance_m"],
        "repeat_payload_error": bool(
            first
            and repeat
            and abs(first["retained_payload_kg"] - repeat["retained_payload_kg"])
            <= cfg["acceptance"]["repeat_mass_tolerance_kg"]
        ),
    }
    checks["low_shallow_acceptance_gate_present"] = (
        "positive_retained_payload" in reports.get("low_shallow", {}).get("checks", {})
    )
    under, under_repeat = (
        reports.get("research_underburden"),
        reports.get("research_underburden_repeat"),
    )
    under_error = None
    under_times_equal = False
    if under and under_repeat:
        with (
            np.load(folder / "research_underburden/recording.npz") as a,
            np.load(folder / "research_underburden_repeat/recording.npz") as b,
        ):
            under_times_equal = np.array_equal(a["time_s"], b["time_s"])
            if a["qpos"].shape == b["qpos"].shape:
                under_error = float(np.max(abs(a["qpos"] - b["qpos"])))
    checks["underburden_repeat"] = bool(
        under
        and under_repeat
        and under_times_equal
        and under_error is not None
        and under_error <= cfg["acceptance"]["repeat_position_tolerance_m"]
        and under["initial_state_sha256"] is not None
        and under["initial_state_sha256"] == under_repeat["initial_state_sha256"]
        and under["prepared_state_sha256"] == under_repeat["prepared_state_sha256"]
        and controlled_factors(under) == controlled_factors(under_repeat)
        and abs(under["retained_payload_kg"] - under_repeat["retained_payload_kg"])
        <= cfg["acceptance"]["repeat_mass_tolerance_kg"]
    )
    checks["default_research_has_geometric_floor_clearance"] = bool(
        under
        and under["checks"]["sequence_completed"]
        and under["reservoir_margins"].get(
            "full_nominal_particle_diameter_below_cutting_edge", False
        )
    )
    rows = []
    for name, r in reports.items():
        f = r["acquisition_funnel"]
        rows.append(
            {
                "scenario": name,
                "status": "COMPLETED_PASS"
                if r["single_run_passed"]
                else "COMPLETED_FAILED_PHYSICS"
                if r["checks"]["sequence_completed"]
                else "INCOMPLETE_OR_FAILED",
                "particles": r["element_count"],
                "initial_mass_kg": r["particle_mass_kg"],
                "settled_initial_mass_kg": r.get("settled_initial_material_mass_kg"),
                "settled_volume_m3": r["initial_pile_envelope"]["volume_m3"]
                if r.get("initial_state_sha256")
                else None,
                "retained_payload_kg": r["retained_payload_kg"]
                if r["checks"]["sequence_completed"]
                else None,
                "retained_particles": r["retained_particle_count"]
                if r["checks"]["sequence_completed"]
                else None,
                "last_observed_bucket_mass_kg": r["retained_payload_kg"],
                "contacted_mass_kg": f["contacted_by_bucket"]["mass_kg"],
                "entered_mouth_mass_kg": f["entered_mouth"]["mass_kg"],
                "acquired_mass_kg": f["acquired_inside_after_mouth_entry"]["mass_kg"],
                "physically_spilled_unique_mass_kg": f[
                    "spilled_during_extraction_unique"
                ]["mass_kg"],
                "displaced_sideways_mass_kg": f[
                    "never_acquired_displaced_sideways_over_diameter"
                ]["mass_kg"],
                "remaining_in_source_mass_kg": f[
                    "never_acquired_remaining_in_source_region"
                ]["mass_kg"],
                "deformation_displaced_mass_kg": r.get(
                    "material_mass_displaced_over_diameter_kg"
                ),
                "peak_bucket_reaction_N": r["peak_bucket_contact_force_N"],
                "max_wheel_slip_m_s": r["maximum_abs_wheel_slip_m_s"],
                "peak_steer_cylinder_N": r["peak_cylinder_force_N"][0],
                "peak_lift_cylinder_N": r["peak_cylinder_force_N"][1],
                "peak_bucket_cylinder_N": r["peak_cylinder_force_N"][2],
                "bulk_fill_factor": r["bulk_equivalent_fill_factor"]
                if r["checks"]["sequence_completed"]
                else None,
                "bucket_sweep_over_pile": r[
                    "bucket_swept_volume_over_initial_pile_volume"
                ],
                "simulated_s": r["cycle_time_s"],
                "wall_s": r["runtime_wall_s"],
                "sim_s_per_wall_s": r["simulation_seconds_per_wall_second"],
                "peak_process_rss_mb": r["peak_process_rss_mb"],
                "stop_reason": r["stop_reason"],
            }
        )
    if rows:
        for name in missing:
            preparation = folder / name / "preparation.json"
            if preparation.exists():
                prep = json.loads(preparation.read_text())
                row = dict.fromkeys(rows[0])
                row.update(
                    scenario=name,
                    status="NO_FINAL_REPORT_CHECK_TIMEOUT_OR_PROGRESS",
                    particles=prep["prepared_particle_count"],
                    initial_mass_kg=prep["prepared_material_mass_kg"],
                    stop_reason="No final dynamics report; prepared mass is not settled initial mass",
                )
                rows.append(row)
        with (folder / "comparison.csv").open("w") as out:
            writer = csv.DictWriter(out, list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    result = {
        "checks": checks,
        "missing_scenarios": missing,
        "m2_cpu_reference_passed": all(checks.values()),
        "isaaclab_newton_gpu_material_execution_verified": False,
        "calibrated_material": False,
        "maximum_repeat_generalized_position_error": position_error,
        "maximum_underburden_repeat_generalized_position_error": under_error,
        "interpretation": "Partial CPU-budget runs cannot establish final payload or pile-size sensitivity. Approximate convex-envelope bulk volumes and voxel cavity sweeps are diagnostics, not calibrated sand properties.",
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
