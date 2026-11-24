"""Compare fixed-reservoir legacy, fresh-feedback and measured-curl trials."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def unchanged_factors(report):
    controller = dict(report["controller"])
    controller.pop("load_response", None)
    return {
        "controller_except_load_response": controller,
        "source": report["source_sha256"],
        "vehicle": report["parameter_sha256"],
        "material": report["material_parameters"],
        "bucket_contact": report["bucket_contact_parameters"],
        "preparation": report["pile_preparation"],
        "solver": report["native_contact_solver"],
    }


def evaluate(folder, baseline):
    paths = {
        "legacy_20hz": baseline,
        "fresh_only": folder / "fresh_only",
        "stop_and_curl": folder / "stop_and_curl",
    }
    if (folder / "stop_and_curl_repeat/report.json").exists():
        paths["stop_and_curl_repeat"] = folder / "stop_and_curl_repeat"
    reports = {k: json.loads((p / "report.json").read_text()) for k, p in paths.items()}
    first = reports["legacy_20hz"]
    checks = {
        "same_vehicle_material_geometry_and_other_controller_parameters": all(
            unchanged_factors(r) == unchanged_factors(first) for r in reports.values()
        ),
        "same_prepared_initial_state": len(
            {r["prepared_state_sha256"] for r in reports.values()}
        )
        == 1,
        "same_settled_initial_state": first["initial_state_sha256"] is not None
        and len({r["initial_state_sha256"] for r in reports.values()}) == 1,
        "fresh_feedback_at_control_rate": all(
            r.get("reaction_feedback_hz") == 240
            for k, r in reports.items()
            if k != "legacy_20hz"
        ),
        "intended_policy_variants": all(
            r["controller"].get("load_response", "legacy")
            == ("stop_and_curl" if k.startswith("stop_and_curl") else "legacy")
            for k, r in reports.items()
        ),
        "candidate_repeat_report_present": "stop_and_curl_repeat" in reports,
        "all_trial_sequences_completed": all(
            r["checks"]["sequence_completed"] for r in reports.values()
        ),
    }
    rows = {}
    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 12))
    for name, p in paths.items():
        r = reports[name]
        events = {e["phase"]: e for e in r["events"]}
        curl_duration = (
            events["curl_advance"]["finished_s"] - events["penetrate"]["finished_s"]
            if "curl_advance" in events
            else None
        )
        rows[name] = {
            "retained_payload_kg": r["retained_payload_kg"],
            "acquisition_funnel": r["acquisition_funnel"],
            "curl_advance_duration_s": curl_duration,
            "reaction_feedback_max_age_s": r.get("reaction_feedback_max_age_s"),
            "peak_bucket_contact_force_N_20hz": r["peak_bucket_contact_force_N"],
            "maximum_mass_residual_kg": r["maximum_mass_residual_kg"],
            "failed_checks": [k for k, v in r["checks"].items() if not v],
            "stop_reason": r["stop_reason"],
            "events": r["events"],
        }
        with np.load(p / "recording.npz") as n:
            t = n["time_s"]
            if "reaction_feedback" in n.files:
                feedback = n["reaction_feedback"]
                rows[name]["maximum_control_observed_reaction_N"] = float(
                    feedback[:, 2].max()
                )
                rows[name]["control_observations_over_trigger"] = int(
                    np.sum(feedback[:, 2] >= r["controller"]["breakout_threshold_N"])
                )
                axes[3].plot(feedback[:, 0], feedback[:, 2] / 1000, label=name)
            else:
                axes[3].plot(t, n["contact_N"][:, 0] / 1000, label=name + " (logged)")
            mass = n["accounting"][
                :, list(n["accounting_columns"]).index("bucket_mass_kg")
            ]
            axes[0].plot(t, mass, label=name)
            axes[1].plot(t, n["cylinder_extension_m"][:, 2], label=name)
            axes[2].plot(t, n["cylinder_extension_m"][:, 1], label=name)
    axes[0].set_ylabel("Measured bucket mass [kg]")
    axes[1].set_ylabel("Curl cylinder extension [m]")
    axes[2].set_ylabel("Lift cylinder extension [m]")
    axes[3].set_ylabel("Bucket contact proxy [kN]")
    axes[3].set_xlabel("Simulation time [s], including settling")
    axes[0].set_ylim(bottom=0, top=max(1.0, axes[0].get_ylim()[1]))
    for ax in axes:
        ax.grid(alpha=0.2)
        ax.legend()
    figure.suptitle("Uncalibrated coarse grains; fixed reservoir/controller ablation")
    figure.tight_layout()
    figure.savefig(folder / "controller_comparison.png", dpi=150)
    plt.close(figure)
    repeat_error = None
    if "stop_and_curl_repeat" in paths:
        with (
            np.load(paths["stop_and_curl"] / "recording.npz") as a,
            np.load(paths["stop_and_curl_repeat"] / "recording.npz") as b,
        ):
            repeat_error = (
                float(np.max(abs(a["qpos"] - b["qpos"])))
                if a["qpos"].shape == b["qpos"].shape
                else None
            )
            checks["candidate_repeat_identical"] = bool(
                np.array_equal(a["time_s"], b["time_s"]) and repeat_error == 0
            )
    compatibility = None
    compatibility_path = folder / "legacy_compatibility/recording.npz"
    if compatibility_path.exists():
        with (
            np.load(baseline.parent / "small_debug/recording.npz") as a,
            np.load(compatibility_path) as b,
        ):
            compatibility = bool(
                np.array_equal(a["time_s"], b["time_s"])
                and np.array_equal(a["qpos"], b["qpos"])
                and np.array_equal(a["masses_kg"], b["masses_kg"])
            )
        checks["legacy_small_debug_trajectory_preserved"] = compatibility
    result = {
        "comparison_valid": all(checks.values()),
        "checks": checks,
        "candidate_repeat_maximum_generalized_position_error": repeat_error,
        "legacy_small_debug_trajectory_preserved": compatibility,
        "results": rows,
        "m2_complete": False,
        "interpretation": "Comparison validity is distinct from physical acquisition acceptance. Uncalibrated CPU reference only; unchanged 20 kN drive-stop trigger and finite cylinder efforts.",
    }
    (folder / "controller_comparison.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument(
        "--baseline",
        type=Path,
        default=Path("outputs/wa320/m2/reservoir_suite/research_underburden"),
    )
    args = parser.parse_args()
    result = evaluate(args.folder, args.baseline)
    print(
        json.dumps(
            {
                "checks": result["checks"],
                "comparison_valid": result["comparison_valid"],
            },
            indent=2,
        )
    )
    if not result["comparison_valid"]:
        raise SystemExit(
            "Controller comparison initial conditions/configuration mismatch"
        )
