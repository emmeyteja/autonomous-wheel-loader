"""Evaluate matched mechanics diagnostics; discrepancies are not machine accuracy."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from wheel_loader.evaluation.backend_benchmark import aligned, trajectory_error


def evaluate(root):
    rows = {}
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    for case in [
        "fixed",
        "ground",
        "fixed_iterations",
        "ground_iterations",
        "fixed_timestep",
        "ground_timestep",
        "fixed_newton",
        "ground_newton",
        "pile_newton",
    ]:
        folder = root / case
        if not (folder / "mjwarp/report.json").exists():
            continue
        manifest = json.loads((folder / "bundle/benchmark.json").read_text())
        reports, states = {}, {}
        for engine in ["native", "mjwarp"]:
            reports[engine] = json.loads((folder / engine / "report.json").read_text())
            if (
                not reports[engine]["completed"]
                or reports[engine]["bundle_sha256"] != manifest["sha256"]
            ):
                raise ValueError("Incomplete/mismatched diagnostic: " + case)
            states[engine] = dict(np.load(folder / engine / "states.npz"))
        a, b = states["native"], states["mjwarp"]
        aligned(a, b)
        error = np.linalg.norm(a["bucket_trajectory"] - b["bucket_trajectory"], axis=1)
        metrics = {
            "bucket_error": trajectory_error(
                a["bucket_trajectory"], b["bucket_trajectory"]
            ),
            "boom_error": trajectory_error(a["boom_trajectory"], b["boom_trajectory"]),
            "first_bucket_discrepancy_over_1mm_s": float(
                a["time_s"][np.flatnonzero(error > 0.001)[0]]
            )
            if np.any(error > 0.001)
            else None,
            "joints": {
                str(n): {
                    "unit": "m" if str(n).endswith("slider_joint") else "rad",
                    "max_abs_discrepancy": float(
                        np.max(abs(a["joint_pos"][:, j] - b["joint_pos"][:, j]))
                    ),
                }
                for j, n in enumerate(a["joint_names"])
            },
            "diagnostic": manifest["diagnostic"],
            "reports": reports,
            "repeat": {},
        }
        if (folder / "physx/report.json").exists():
            pr = json.loads((folder / "physx/report.json").read_text())
            if not pr["completed"] or pr["bundle_sha256"] != manifest["sha256"]:
                raise ValueError("Incomplete/mismatched PhysX diagnostic")
            pn = dict(np.load(folder / "physx/states.npz"))
            aligned(a, pn)
            metrics["physx_feasibility"] = {
                "bucket_error": trajectory_error(
                    a["bucket_trajectory"], pn["bucket_trajectory"]
                ),
                "matched_collision_geometry": False,
                "report": pr,
            }
        for engine in ["native", "mjwarp"]:
            repeat = folder / (engine + "_repeat")
            if (repeat / "report.json").exists():
                rr = json.loads((repeat / "report.json").read_text())
                if not rr["completed"] or rr["bundle_sha256"] != manifest["sha256"]:
                    raise ValueError("Mismatched repeat")
                r = dict(np.load(repeat / "states.npz"))
                aligned(states[engine], r)
                metrics["repeat"][engine] = float(
                    np.linalg.norm(
                        states[engine]["body_positions"] - r["body_positions"], axis=-1
                    ).max()
                )
        rows[case] = metrics
        if case != "pile_newton":
            row = 0 if case.startswith("fixed") else 1
            axes[row, 0].plot(a["time_s"], error * 1000, label=case)
            for engine, n in states.items():
                if case in {"fixed", "ground", "fixed_newton", "ground_newton"}:
                    axes[row, 1].plot(
                        n["time_s"],
                        n["bucket_trajectory"][:, 2],
                        label=case + " / " + engine,
                    )
    for i, label in enumerate(["Fixed rear, contact disabled", "Unloaded ground"]):
        axes[i, 0].set_ylabel("Bucket discrepancy [mm]")
        axes[i, 1].set_ylabel("Bucket origin Z [m]")
        for ax in axes[i]:
            ax.set_title(label)
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7)
            ax.set_xlabel("Simulation time [s]")
    if not rows:
        plt.close(fig)
        raise ValueError("No completed matched diagnostics found")
    fig.suptitle(
        "Matched numerical changes; same physical effort tape — sim-to-sim only"
    )
    fig.tight_layout()
    fig.savefig(root / "mechanics_comparison.png", dpi=150)
    plt.close(fig)
    sensitivity = {}
    for name in ["settle_cg", "settle_perturbed"]:
        path = root / name / "native/states.npz"
        if path.exists():
            sensitivity[name] = dict(np.load(path))
    native_sensitivity = None
    if len(sensitivity) == 2:
        a, b = sensitivity["settle_cg"], sensitivity["settle_perturbed"]
        aligned(a, b)
        manifests = {
            name: json.loads((root / name / "bundle/benchmark.json").read_text())
            for name in sensitivity
        }
        for name, manifest in manifests.items():
            report = json.loads((root / name / "native/report.json").read_text())
            if not report["completed"] or report["bundle_sha256"] != manifest["sha256"]:
                raise ValueError("Incomplete/mismatched sensitivity run")
        native_sensitivity = {
            "purpose": "deliberately different initial velocity; not a matched backend comparison",
            "curl_velocity_perturbation_m_s": manifests["settle_perturbed"][
                "diagnostic"
            ]["initial_curl_velocity_perturbation_m_s"],
            "bucket_error": trajectory_error(
                a["bucket_trajectory"], b["bucket_trajectory"]
            ),
        }
    tight = root / "ground_tight/native/states.npz"
    native_tolerance = None
    if tight.exists():
        a = dict(np.load(tight))
        b = dict(np.load(root / "ground_newton/native/states.npz"))
        aligned(a, b)
        native_tolerance = {
            "purpose": "native-only CG tolerance study; GPU clamps tolerance to at least 1e-6",
            "bucket_error_vs_native_newton": trajectory_error(
                a["bucket_trajectory"], b["bucket_trajectory"]
            ),
        }
    result = {
        "comparison": "sim-to-sim only; no real WA320 accuracy validation",
        "cases": rows,
        "native_initial_velocity_sensitivity": native_sensitivity,
        "native_tighter_tolerance": native_tolerance,
    }
    (root / "comparison.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--root", type=Path, default=Path("outputs/wa320/mechanics_benchmark")
    )
    evaluate(p.parse_args().root)
