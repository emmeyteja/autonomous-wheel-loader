"""Summarize bounded backend replays; do not rank physical accuracy."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from wheel_loader.environment.simulation.backend_benchmark import (
    load_bundle,
    validate_parameters,
)
from wheel_loader.evaluation.backend_benchmark import compare, envelope_penetration
from wheel_loader.vehicle.parameters import load_parameters


def evaluate(root):
    model, tape, manifest = load_bundle(root / "bundle")
    parameters = load_parameters("configs/vehicle/wa320.yaml")
    validate_parameters(parameters, manifest)
    states, reports = {}, {}
    for name in ("native", "mjwarp", "physx"):
        reports[name] = json.loads((root / name / "report.json").read_text())
        if not reports[name]["completed"]:
            raise ValueError("Incomplete replay: " + name)
        if reports[name]["bundle_sha256"] != manifest["sha256"]:
            raise ValueError("Different command/model bundle: " + name)
        states[name] = dict(np.load(root / name / "states.npz"))
    ref = states["native"]
    grains = ref["grain_body_ids"]
    settled = ref["body_positions"][np.searchsorted(ref["time_s"], 10), grains]
    radii = np.array(
        [model.geom_size[np.flatnonzero(model.geom_bodyid == b)[0], 0] for b in grains]
    )
    summary = {
        "comparison": manifest["comparison"],
        "bundle_sha256": manifest["sha256"],
        "backends": {},
    }
    fig, axes = plt.subplots(3, 2, figsize=(13, 11), sharex=True)
    for name, n in states.items():
        r = reports[name]
        repeated = dict(np.load(root / (name + "_repeat") / "states.npz"))
        rr = json.loads((root / (name + "_repeat") / "report.json").read_text())
        if not rr["completed"] or rr["bundle_sha256"] != manifest["sha256"]:
            raise ValueError("Incomplete or mismatched repeat: " + name)
        metrics = compare(ref, n, repeated)
        depth = envelope_penetration(
            n["lip_trajectory"], settled, radii, parameters["vehicle"]["bucket_width"]
        )
        active = np.isin(n["phase"], ["penetrate", "curl"])
        valid = active & np.isfinite(depth)
        metrics["initial_envelope_penetration_max_m"] = (
            float(np.max(depth[valid])) if valid.any() else None
        )
        metrics["penetration_valid_samples"] = int(valid.sum())
        metrics["penetration_definition"] = (
            "common native-settled sphere envelope, world-Y edge-strip proxy; +X approach only"
        )
        metrics["max_joint_repeat_difference"] = {
            str(j): float(
                np.max(abs(n["joint_pos"][:, k] - repeated["joint_pos"][:, k]))
            )
            for k, j in enumerate(n["joint_names"])
        }
        metrics["joint_state_ranges"] = {
            str(j): {
                "min": float(n["joint_pos"][:, k].min()),
                "max": float(n["joint_pos"][:, k].max()),
                "unit": "m" if str(j).endswith("slider_joint") else "rad",
            }
            for k, j in enumerate(n["joint_names"])
        }
        forces = r.get("contact_forces_available", True)
        if not forces:
            r["peak_bucket_reaction_N"] = None
        metrics["peak_wheel_ground_reaction_N"] = (
            float(np.linalg.norm(n["wheel_ground_reaction_N"], axis=-1).max())
            if forces
            else None
        )
        columns = list(n["accounting_columns"])
        metrics["final_accounting_kg"] = dict(
            zip(columns, n["accounting"][-1].tolist())
        )
        metrics["max_rear_tilt_rad"] = float(
            np.arccos(np.clip(n["body_rotations"][:, 1, 2, 2], -1, 1)).max()
        )
        summary["backends"][name] = {**r, **metrics}
        t = n["time_s"]
        label = name + (" (non-equivalent)" if name == "physx" else "")
        axes[0, 0].plot(t, n["bucket_trajectory"][:, 2], label=label)
        axes[0, 1].plot(t, n["boom_trajectory"][:, 2], label=label)
        axes[1, 0].plot(t, n["articulation_rad"], label=label)
        axes[1, 1].plot(
            t, n["accounting"][:, columns.index("bucket_mass_kg")], label=label
        )
        if forces:
            axes[2, 0].plot(t, n["bucket_reaction_N"], label=label)
        axes[2, 1].plot(t, np.where(active, depth, np.nan), label=label)
    for ax, title in zip(
        axes.flat,
        [
            "Bucket origin Z [m]",
            "Boom origin Z [m]",
            "Articulation [rad]",
            "Material inside bucket [kg]",
            "Sampled bucket/grain reaction [N]",
            "Initial-envelope penetration proxy [m]",
        ],
    ):
        ax.set_ylabel(title)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Time [s]")
    fig.suptitle("Identical open-loop commands — sim-to-sim, not real WA320 accuracy")
    fig.tight_layout()
    fig.savefig(root / "comparison.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    names = list(ref["joint_names"])
    for name, n in states.items():
        axes[0].plot(
            n["time_s"],
            n["joint_pos"][:, names.index("lift_left_slider_joint")],
            label=name,
        )
        axes[1].plot(
            n["time_s"],
            n["joint_pos"][:, names.index("bucket_slider_joint")],
            label=name,
        )
    axes[0].set_ylabel("Left lift-cylinder coordinate [m]")
    axes[1].set_ylabel("Curl-cylinder coordinate [m]")
    axes[1].set_xlabel("Time [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(root / "cylinder_states.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for name, n in states.items():
        axes[0].scatter(n["final_pile"][:, 0], n["final_pile"][:, 2], s=14, label=name)
    axes[0].scatter(
        settled[:, 0], settled[:, 2], s=14, label="Common settled reference", marker="x"
    )
    axes[0].set(
        xlabel="World X [m]",
        ylabel="World Z [m]",
        title="Final material centres (all IDs)",
    )
    axes[0].legend()
    axes[0].set_aspect("equal", adjustable="datalim")
    time = (np.arange(len(tape["effort"])) + 1) * manifest["control_dt_s"]
    for a in range(model.nu):
        axes[1].plot(time, tape["effort"][:, a], label=str(a))
    axes[1].set(
        xlabel="Time [s]",
        ylabel="Physical force [N] / torque [Nm]",
        title="Identical nine-channel effort tape",
    )
    axes[1].legend(title="Channel", ncol=3)
    fig.tight_layout()
    fig.savefig(root / "pile_and_commands.png", dpi=150)
    plt.close(fig)
    (root / "comparison.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n"
    )
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, default=Path("outputs/wa320/backend_benchmark")
    )
    args = parser.parse_args()
    evaluate(args.root)
