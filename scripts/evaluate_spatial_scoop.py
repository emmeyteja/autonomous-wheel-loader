"""Evaluate geometry feedback trials without turning failed runs into successes."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from wheel_loader.environment.simulation.backend_benchmark import load_bundle
from wheel_loader.environment.simulation.spatial_scoop import SpatialScoopFeedback
from wheel_loader.vehicle.parameters import load_parameters


def evaluate(root):
    result = {
        "scope": "simulation diagnostics; uncalibrated material, no real-machine accuracy claim",
        "cases": {},
    }
    for case in ["unloaded", "loaded"]:
        rows = {}
        states = {}
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        for folder in sorted((root / case).glob("*")):
            if not (folder / "report.json").exists():
                continue
            report = json.loads((folder / "report.json").read_text())
            model, _, manifest = load_bundle(folder / "bundle")
            if report["bundle_sha256"] != manifest["sha256"]:
                raise ValueError("Run input hashes differ")
            s = dict(np.load(folder / "states.npz"))
            c = np.load(folder / "controller.npz")["values"]
            cfg = report["controller_configuration"]
            observer = SpatialScoopFeedback(
                model, load_parameters("configs/vehicle/wa320.yaml"), cfg
            )
            rotation = s["body_rotations"][:, observer.bucket]
            position = s["body_positions"][:, observer.bucket]
            clearance = (
                np.einsum("ni,vi->nv", rotation[:, 2, :], observer.vertices)
                + position[:, 2, None]
            ).min(axis=1)
            scalar = observer.joints
            limited = model.jnt_limited[scalar].astype(bool)
            q = s["joint_pos"][:, limited]
            ranges = model.jnt_range[scalar[limited]]
            overrun = float(
                np.maximum(ranges[:, 0] - q, q - ranges[:, 1]).max(initial=0)
            )
            tilt = np.arccos(
                np.clip(s["body_rotations"][:, observer.rear, 2, 2], -1, 1)
            )
            forces_finite = all(
                np.isfinite(s[n]).all()
                for n in [
                    "actuator_force",
                    "generalized_constraint_force",
                    "wheel_ground_reaction_N",
                    "bucket_reaction_N",
                ]
            )
            acct = {
                str(k): s["accounting"][:, i]
                for i, k in enumerate(s["accounting_columns"])
            }
            residual = float(abs(acct["conservation_residual_kg"]).max())
            gates = {
                "controller_sequence_completed": bool(
                    report["controller_done"] and not report["controller_failed"]
                ),
                "positive_ground_clearance": bool(
                    clearance.min() >= cfg["minimum_clearance_m"] - 0.001
                ),
                "joint_limits": overrun <= 0.001,
                "pin_constraints": report["max_pin_gap_m"] <= 0.001,
                "vehicle_tilt": bool(tilt.max() <= np.deg2rad(12)),
                "forces_finite": forces_finite,
                "capacity_and_solver": not any(
                    report.get("native_warning_counts_all_steps") or []
                )
                and (report.get("gpu_all_step_overflow_union") or 0) == 0,
                "mass_accounting": residual
                <= 1e-9 * max(1, float(acct["initial_mass_kg"][0])),
                "retained_payload": case == "unloaded"
                or report["retained_material_kg"] >= cfg["minimum_payload_kg"],
            }
            av = [
                int(np.flatnonzero(scalar == j)[0]) for j in model.actuator_trnid[:, 0]
            ]
            power = (s["actuator_force"] * s["joint_velocity"][:, av]).sum(axis=1)
            dt = manifest["control_dt_s"]
            row = {
                "passed": all(gates.values()),
                "gates": gates,
                "controller_reason": report["controller_reason"],
                "duration_s": report["simulated_s"],
                "minimum_floor_clearance_m": float(clearance.min()),
                "stroke_or_angle_overrun_mixed_SI": overrun,
                "max_tilt_deg": float(np.rad2deg(tilt.max())),
                "retained_kg": float(acct["bucket_mass_kg"][-1]),
                "peak_bucket_mass_kg": float(acct["bucket_mass_kg"].max()),
                "cumulative_acquired_kg": float(
                    acct["cumulative_acquired_mass_kg"][-1]
                ),
                "spilled_kg": float(acct["spilled_mass_kg"][-1]),
                "mass_conservation_error_kg": residual,
                "mechanical_net_work_J": float(power.sum() * dt),
                "positive_mechanical_work_J": float(np.maximum(power, 0).sum() * dt),
                "peak_bucket_reaction_N": float(s["bucket_reaction_N"].max()),
                "peak_actuator_force_or_torque": abs(s["actuator_force"])
                .max(axis=0)
                .tolist(),
                "input_hashes": manifest["sha256"],
                "controller_configuration": cfg,
                "events": report["events"],
            }
            rows[folder.name] = row
            states[folder.name] = s
            axes[0, 0].plot(s["time_s"], s["lip_trajectory"][:, 2], label=folder.name)
            axes[0, 1].plot(s["time_s"], clearance * 1000, label=folder.name)
            axes[1, 0].plot(s["time_s"], acct["bucket_mass_kg"], label=folder.name)
            axes[1, 1].plot(c[:, 0], c[:, 13] / 1000, label=folder.name)
            (folder / "validation.json").write_text(
                json.dumps(row, indent=2, allow_nan=False) + "\n"
            )
        if not rows:
            plt.close(fig)
            continue
        baseline = states.get("native_1")
        comparison = {}
        if baseline is not None:
            for name, s in states.items():
                if (
                    rows[name]["input_hashes"] != rows["native_1"]["input_hashes"]
                    or rows[name]["controller_configuration"]
                    != rows["native_1"]["controller_configuration"]
                ):
                    raise ValueError("Comparison inputs/controllers differ")
                n = min(len(s["time_s"]), len(baseline["time_s"]))
                if not np.array_equal(s["time_s"][:n], baseline["time_s"][:n]):
                    raise ValueError("Control timestamps differ")
                comparison[name] = {
                    "max_bucket_discrepancy_on_common_interval_m": float(
                        np.linalg.norm(
                            s["bucket_trajectory"][:n]
                            - baseline["bucket_trajectory"][:n],
                            axis=1,
                        ).max()
                    ),
                    "max_body_discrepancy_on_common_interval_m": float(
                        np.linalg.norm(
                            s["body_positions"][:n] - baseline["body_positions"][:n],
                            axis=-1,
                        ).max()
                    ),
                    "duration_difference_s": rows[name]["duration_s"]
                    - rows["native_1"]["duration_s"],
                }
        result["cases"][case] = {
            "runs": rows,
            "comparison_against_native_1": comparison,
            "three_native_repeats_present": all(
                "native_" + str(i) in rows for i in [1, 2, 3]
            ),
        }
        for ax, title in zip(
            axes.flat,
            [
                "Cutting-edge height [m]",
                "Bucket floor clearance [mm]",
                "In-bucket mass [kg]",
                "Bucket/material reaction [kN]",
            ],
        ):
            ax.set(title=title, xlabel="Simulation time [s]")
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7)
        fig.suptitle(case + " geometry-feedback tests — simulation only")
        fig.tight_layout()
        fig.savefig(root / (case + "_metrics.png"), dpi=150)
        plt.close(fig)
    (root / "comparison.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--root", type=Path, default=Path("outputs/wa320/spatial_scoop/shared_v3")
    )
    a = p.parse_args()
    evaluate(a.root)
