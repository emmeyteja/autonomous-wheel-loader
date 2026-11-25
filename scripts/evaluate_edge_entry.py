"""Compare recorded fixture repetitions and plot physical contact diagnostics."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def evaluate(root):
    baseline = root / "repeat_1"
    results = {}
    for count in (1, 3):
        fig, axes = plt.subplots(3, 2, figsize=(11, 10))
        for name in (
            f"row_{count}_straight",
            f"row_{count}_curl_040",
            f"row_{count}_curl_095",
        ):
            if not (baseline / name / "report.json").exists():
                continue
            state = np.load(baseline / name / "states.npz")
            report = json.loads((baseline / name / "report.json").read_text())
            repeat = {}
            for folder in sorted(root.glob("repeat_*")):
                other = np.load(folder / name / "states.npz")
                repeat[folder.name] = {
                    key: bool(np.array_equal(state[key], other[key]))
                    for key in ["qpos", "contacts", "accounting", "carriage_effort"]
                }
            report["repetitions_identical"] = repeat
            report["contact_normal_elevation_first_deg"] = float(
                np.rad2deg(
                    np.arcsin(report["first_contact_normal_on_particle_world"][2])
                )
            )
            gates = {
                "three_recorded_repetitions": len(repeat) == 3,
                "repeatable": all(all(v.values()) for v in repeat.values()),
                "finite_forces": report["sampled_forces_finite"],
                "finite_state": bool(np.isfinite(state["qpos"]).all()),
                "clear_floor": report["minimum_bucket_floor_clearance_m"] >= 0.04,
                "bounded_tracking": bool(
                    (np.max(abs(state["tracking_error"]), 0) < [0.01, 0.01, 0.01]).all()
                ),
                "force_limit": bool(abs(state["carriage_effort"]).max() <= 3e5),
                "particle_floor_overlap": report["maximum_particle_floor_overlap_m"]
                <= 0.001,
                "mass_accounting": report["maximum_accounting_error_kg"] < 1e-9,
                "no_solver_warning": not any(report["solver_warnings"]),
            }
            report["diagnostic_gates"] = gates
            report["diagnostic_valid"] = all(gates.values())
            report["scoop_success"] = (
                False  # No lift/reverse/retention sequence in this fixture.
            )
            results[name] = report
            t = state["time_s"]
            axes[0, 0].plot(t, state["xyz"][:, :, 0].mean(1) - 0.5, label=name)
            axes[0, 1].plot(t, state["xyz"][:, :, 2].mean(1), label=name)
            axes[1, 0].plot(t, state["force_x_N"], label=name)
            axes[1, 1].plot(t, state["force_z_N"], label=name)
            contacts = state["contacts"]
            axes[2, 0].scatter(
                contacts[:, 0],
                np.rad2deg(np.arcsin(np.clip(contacts[:, 7], -1, 1))),
                s=1,
                label=name,
            )
            axes[2, 1].plot(t, state["bucket_ground_clearance_m"] * 1000, label=name)
        for ax, title in zip(
            axes.flat,
            [
                "Particle forward displacement [m]",
                "Particle centre height [m]",
                "Bucket force on particles: X [N]",
                "Bucket force on particles: Z [N]",
                "Contact-normal elevation: world X/Z [deg]",
                "Bucket/floor clearance [mm]",
            ],
        ):
            ax.set(title=title, xlabel="Time [s]")
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7)
        fig.suptitle(
            f"{count} particle(s): isolated finite-force carriage, not a WA320 scoop"
        )
        fig.tight_layout()
        fig.savefig(root / f"row_{count}_contact.png", dpi=150)
        plt.close(fig)
    (root / "evaluation.json").write_text(
        json.dumps(results, indent=2, allow_nan=False) + "\n"
    )
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", type=Path, required=True)
    a = p.parse_args()
    result = evaluate(a.root)
    if not all(r["diagnostic_valid"] for r in result.values()):
        raise SystemExit("Fixture diagnostic gates failed; see evaluation.json")
