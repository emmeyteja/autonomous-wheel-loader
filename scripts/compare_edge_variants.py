"""Matched geometry/clearance comparison; fixture evidence, not M2 acceptance."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def compare(root):
    results = {}
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for row, height in enumerate([0.085, 0.075]):
        folder = root / f"h{height}"
        baseline = json.loads((folder / "blunt/evaluation.json").read_text())
        for variant in ["blunt", "taper_5mm", "taper_15mm"]:
            cases = json.loads((folder / variant / "evaluation.json").read_text())
            for case, report in cases.items():
                base = baseline[case]
                # Physical source, initial particle/vehicle coordinates and control
                # law must match within each geometry comparison, not just its label.
                for field in [
                    "source_compiled_sha256",
                    "particle_count",
                    "particle_radius_m",
                    "initial_mass_kg",
                    "attack_height_m",
                    "curl_start_entry_m",
                    "duration_s",
                    "timestep_s",
                    "observation_hz",
                    "carriage_servo_Kp_N_per_m_Nm_per_rad",
                    "carriage_servo_Kd_Ns_per_m_Nms_per_rad",
                    "carriage_generalized_force_limits_N_N_Nm",
                ]:
                    if report[field] != base[field]:
                        raise ValueError(f"Unmatched {field}")
                state = np.load(folder / variant / "repeat_1" / case / "states.npz")
                original = np.load(folder / "blunt/repeat_1" / case / "states.npz")
                np.testing.assert_array_equal(state["qpos"][0], original["qpos"][0])
                report["matched_initial_state_and_controller"] = True
                results[f"h{height}/{variant}/{case}"] = report
            name = "row_3_straight"
            state = np.load(folder / variant / "repeat_1" / name / "states.npz")
            axes[row, 0].plot(
                state["time_s"], state["xyz"][:, :, 2].mean(1), label=variant
            )
            axes[row, 1].plot(state["time_s"], state["inside"].sum(1), label=variant)
        axes[row, 0].set(
            title=f"Reference lip height {height:.3f} m: particle centre height",
            ylabel="Height [m]",
            xlabel="Time [s]",
        )
        axes[row, 1].set(
            title=f"Reference lip height {height:.3f} m: inside bucket",
            ylabel="Particle count",
            xlabel="Time [s]",
        )
    for ax in axes.flat:
        ax.grid(alpha=0.3)
        ax.legend()
    fig.suptitle(
        "Collision-only taper sensitivity: three-particle straight entry; hypothetical geometry"
    )
    fig.tight_layout()
    fig.savefig(root / "variant_comparison.png", dpi=150)
    plt.close(fig)
    legacy = Path("outputs/wa320/edge_entry/finite_final/repeat_1")
    unchanged = {}
    if legacy.exists():
        for name in baseline:
            previous = np.load(legacy / name / "states.npz")
            current = np.load(root / "h0.085/blunt/repeat_1" / name / "states.npz")
            unchanged[name] = all(
                np.array_equal(previous[k], current[k])
                for k in ["qpos", "contacts", "accounting", "carriage_effort"]
            )
    summary = {
        "scope": "Isolated carriage geometry sensitivity; not a WA320 scoop",
        "legacy_blunt_traces_identical": unchanged,
        "cases": results,
    }
    (root / "matched_comparison.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n"
    )
    return summary


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", type=Path, required=True)
    a = p.parse_args()
    compare(a.root)
