"""Evaluate constraint-consistent sensitivity; numerical discrepancies are not accuracy."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from wheel_loader.environment.simulation.backend_benchmark import load_bundle
from wheel_loader.evaluation.backend_benchmark import aligned, trajectory_error


def sampled_grain_vehicle_contacts(bundle, states):
    """Reconstruct collision geometry at saved poses only; no force inference."""
    import mujoco

    model, _, _ = load_bundle(bundle)
    data = mujoco.MjData(model)
    grains = set(states["grain_body_ids"].tolist())
    frames = 0
    for q in states["qpos"]:
        data.qpos[:] = q
        mujoco.mj_fwdPosition(model, data)
        hit = False
        for contact in data.contact[: data.ncon]:
            a, b = map(int, model.geom_bodyid[contact.geom])
            if (a in grains and b != 0 and b not in grains) or (
                b in grains and a != 0 and a not in grains
            ):
                hit = True
                break
        frames += hit
    return int(frames)


def read(folder):
    report = json.loads((folder / "report.json").read_text())
    if not report["completed"]:
        raise ValueError("Incomplete run: " + str(folder))
    return (
        report,
        dict(np.load(folder / "states.npz")),
        dict(np.load(folder / "mechanism.npz")),
    )


def summary(states, audit):
    sv = audit["equality_singular_values"]
    rank = (sv > 1e-9).sum(axis=1)
    positive = np.where(sv > 1e-9, sv, np.inf).min(axis=1)
    sliders = [
        i
        for i, n in enumerate(states["joint_names"])
        if str(n).endswith("slider_joint")
    ]
    finite_wheel = np.isfinite(states["wheel_ground_reaction_N"])
    finite_bucket = np.isfinite(states["bucket_reaction_N"])
    return {
        "max_equality_position_component_m": float(
            abs(audit["equality_position"]).max()
        ),
        "max_equality_velocity_component_m_s": float(
            abs(audit["equality_velocity"]).max()
        ),
        "max_abs_generalized_constraint_force_by_dof": abs(
            audit["generalized_constraint_force"]
        )
        .max(axis=0)
        .tolist(),
        "equality_jacobian_rank_min_max": [int(rank.min()), int(rank.max())],
        "minimum_retained_singular_value_mixed_SI": float(positive.min()),
        "energy_range_J": np.stack(
            [
                audit["energy_potential_kinetic_J"].min(axis=0),
                audit["energy_potential_kinetic_J"].max(axis=0),
            ]
        ).tolist(),
        "max_contact_count": int(states["contact_count"].max()),
        "nonfinite_wheel_reaction_components": int((~finite_wheel).sum()),
        "nonfinite_bucket_reaction_samples": int((~finite_bucket).sum()),
        "max_bucket_grain_reaction_N": float(states["bucket_reaction_N"].max())
        if finite_bucket.all()
        else None,
        "max_wheel_ground_reaction_N": float(
            np.linalg.norm(states["wheel_ground_reaction_N"], axis=-1).max()
        )
        if finite_wheel.all()
        else None,
        "cylinders": {
            str(states["joint_names"][i]): {
                "position_min_max_m": [
                    float(states["joint_pos"][:, i].min()),
                    float(states["joint_pos"][:, i].max()),
                ],
                "max_speed_m_s": float(abs(states["joint_velocity"][:, i]).max()),
            }
            for i in sliders
        },
    }


def evaluate(root):
    results = {
        "comparison": "sim-to-sim only; no physical instability or machine accuracy claim",
        "sensitivity": {},
        "full_newton": {},
        "feedback": {},
    }
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    for ci, case in enumerate(["fixed", "ground", "pile"]):
        base = root / case / "baseline"
        manifest = json.loads((base / "bundle/benchmark.json").read_text())
        _, baseline_tape, _ = load_bundle(base / "bundle")
        ref, a, aa = read(base / "native_1")
        repeats = []
        for rep in [2, 3]:
            rr, b, _bb = read(base / f"native_{rep}")
            if rr["bundle_sha256"] != ref["bundle_sha256"]:
                raise ValueError("Repeat input mismatch")
            aligned(a, b)
            repeats.append(
                float(
                    np.linalg.norm(
                        a["body_positions"] - b["body_positions"], axis=-1
                    ).max()
                )
            )
        row = {
            "initial_states": manifest["investigation"]["initial_states"],
            "projection": manifest["investigation"]["projection"],
            "baseline_repeat_max_body_difference_m": repeats,
            "baseline_summary": summary(a, aa),
            "perturbations": {},
        }
        if case == "pile":
            row["baseline_sampled_grain_vehicle_contact_frames"] = (
                sampled_grain_vehicle_contacts(base / "bundle", a)
            )
        for mode in ["isolated", "consistent"]:
            _, perturbed_tape, perturbed_manifest = load_bundle(
                root / case / mode / "bundle"
            )
            if (
                perturbed_manifest["sha256"]["compiled_model.mjb"]
                != manifest["sha256"]["compiled_model.mjb"]
                or not np.array_equal(
                    baseline_tape["initial_qpos"], perturbed_tape["initial_qpos"]
                )
                or not np.array_equal(baseline_tape["effort"], perturbed_tape["effort"])
            ):
                raise ValueError(
                    "Sensitivity experiment changed model, pose or effort tape"
                )
            _, b, ba = read(root / case / mode / "native_1")
            aligned(a, b)
            err = np.linalg.norm(
                a["bucket_trajectory"] - b["bucket_trajectory"], axis=1
            )
            row["perturbations"][mode] = {
                "bucket": trajectory_error(
                    a["bucket_trajectory"], b["bucket_trajectory"]
                ),
                "boom": trajectory_error(a["boom_trajectory"], b["boom_trajectory"]),
                "first_over_1mm_s": float(a["time_s"][np.flatnonzero(err > 0.001)[0]])
                if np.any(err > 0.001)
                else None,
                "summary": summary(b, ba),
            }
            if case == "pile" and mode == "consistent":
                row["consistent_sampled_grain_vehicle_contact_frames"] = (
                    sampled_grain_vehicle_contacts(root / case / mode / "bundle", b)
                )
            axes[ci].plot(a["time_s"], np.maximum(err, 1e-12), label="CG / " + mode)
        for suffix, label in [("newton", "Newton"), ("passive", "CG / zero efforts")]:
            folder = root / (case + "_" + suffix)
            if (folder / "consistent/native_1/report.json").exists():
                _, b, _ = read(folder / "baseline/native_1")
                _, c, ca = read(folder / "consistent/native_1")
                aligned(b, c)
                row[suffix + "_consistent"] = {
                    "bucket": trajectory_error(
                        b["bucket_trajectory"], c["bucket_trajectory"]
                    ),
                    "summary": summary(c, ca),
                }
                axes[ci].plot(
                    b["time_s"],
                    np.maximum(
                        np.linalg.norm(
                            b["bucket_trajectory"] - c["bucket_trajectory"], axis=1
                        ),
                        1e-12,
                    ),
                    label=label + " / consistent",
                )
        results["sensitivity"][case] = row
        axes[ci].set(
            title=case, xlabel="Time [s]", ylabel="Bucket discrepancy [m]", yscale="log"
        )
        axes[ci].legend(fontsize=7)
        axes[ci].grid(alpha=0.3)
    floating = root / "floating_no_contact"
    if (floating / "consistent/native_1/report.json").exists():
        _, a, _ = read(floating / "baseline/native_1")
        _, b, ba = read(floating / "consistent/native_1")
        aligned(a, b)
        results["floating_no_contact"] = {
            "bucket": trajectory_error(a["bucket_trajectory"], b["bucket_trajectory"]),
            "summary": summary(b, ba),
        }
        axes[1].plot(
            a["time_s"],
            np.maximum(
                np.linalg.norm(a["bucket_trajectory"] - b["bucket_trajectory"], axis=1),
                1e-12,
            ),
            linestyle="--",
            label="CG / floating, contacts off",
        )
        axes[1].legend(fontsize=7)
    fig.suptitle("Initial bucket-cylinder speed +1e-7 m/s; unchanged release geometry")
    fig.tight_layout()
    fig.savefig(root / "sensitivity.png", dpi=160)
    plt.close(fig)
    full = root / "full_newton"
    if all(
        (full / f"{e}_{i}/report.json").exists()
        for e in ["native", "mjwarp"]
        for i in [1, 2]
    ):
        reports = {}
        states = {}
        audits = {}
        for engine in ["native", "mjwarp"]:
            report, a, aa = read(full / f"{engine}_1")
            rr, b, _bb = read(full / f"{engine}_2")
            if report["bundle_sha256"] != rr["bundle_sha256"]:
                raise ValueError("Full repeat input mismatch")
            aligned(a, b)
            reports[engine] = report
            states[engine] = a
            audits[engine] = aa
            results["full_newton"][engine] = {
                "report": report,
                "repeat_report": rr,
                "summary": summary(a, aa),
                "repeat_max_body_difference_m": float(
                    np.linalg.norm(
                        a["body_positions"] - b["body_positions"], axis=-1
                    ).max()
                ),
                "repeat_bucket": trajectory_error(
                    a["bucket_trajectory"], b["bucket_trajectory"]
                ),
                "overflow_union_both_runs": [
                    report["gpu_all_step_overflow_union"],
                    rr["gpu_all_step_overflow_union"],
                ],
            }
        if reports["native"]["bundle_sha256"] != reports["mjwarp"]["bundle_sha256"]:
            raise ValueError("Cross-engine inputs differ")
        a, b = states["native"], states["mjwarp"]
        _, second_gpu, _ = read(full / "mjwarp_2")
        results["full_newton"]["mjwarp"]["contact_force_export_valid"] = False
        results["full_newton"]["mjwarp"]["contact_force_export_limitation"] = (
            "Installed host bridge assigns force addresses to inactive contacts. Exported contact forces are withheld; see five-tick contact export reproducer. Dynamics and trajectory data remain separate."
        )
        results["full_newton"]["mjwarp"]["summary"]["max_bucket_grain_reaction_N"] = (
            None
        )
        results["full_newton"]["mjwarp"]["summary"]["max_wheel_ground_reaction_N"] = (
            None
        )
        aligned(a, b)
        results["full_newton"]["cross_engine"] = {
            "second_gpu_vs_native_bucket": trajectory_error(
                a["bucket_trajectory"], second_gpu["bucket_trajectory"]
            ),
            "bucket": trajectory_error(a["bucket_trajectory"], b["bucket_trajectory"]),
            "boom": trajectory_error(a["boom_trajectory"], b["boom_trajectory"]),
            "actuated_joint_max_discrepancy": {
                str(n): float(abs(a["joint_pos"][:, i] - b["joint_pos"][:, i]).max())
                for i, n in enumerate(a["joint_names"])
                if str(n).endswith("slider_joint") or str(n) == "articulation_joint"
            },
        }
        import mujoco

        model, _, _ = load_bundle(full / "bundle")
        rear = next(
            i
            for i in range(model.nbody)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").endswith(
                "_rear_frame"
            )
        )
        local = [
            np.einsum(
                "nji,nj->ni",
                s["body_rotations"][:, rear],
                s["bucket_trajectory"] - s["body_positions"][:, rear],
            )
            for s in [a, b]
        ]
        results["full_newton"]["cross_engine"]["rear_frame_position"] = (
            trajectory_error(a["body_positions"][:, rear], b["body_positions"][:, rear])
        )
        results["full_newton"]["cross_engine"]["bucket_in_rear_frame"] = (
            trajectory_error(*local)
        )
        fig, axes = plt.subplots(2, 2, figsize=(13, 8))
        for e, s in states.items():
            axes[0, 0].plot(s["time_s"], s["bucket_trajectory"][:, 2], label=e)
            if e == "native":
                axes[1, 0].plot(s["time_s"], s["bucket_reaction_N"] / 1000, label=e)
            axes[1, 1].plot(
                np.r_[0, s["time_s"]],
                abs(audits[e]["equality_velocity"]).max(axis=1),
                label=e,
            )
        axes[0, 1].plot(
            a["time_s"],
            np.linalg.norm(a["bucket_trajectory"] - b["bucket_trajectory"], axis=1),
            label="native vs GPU 1",
        )
        axes[0, 0].plot(
            second_gpu["time_s"],
            second_gpu["bucket_trajectory"][:, 2],
            linestyle="--",
            label="mjwarp repeat",
        )
        axes[0, 1].plot(
            a["time_s"],
            np.linalg.norm(
                a["bucket_trajectory"] - second_gpu["bucket_trajectory"], axis=1
            ),
            linestyle="--",
            label="native vs GPU repeat",
        )
        axes[0, 1].legend()
        axes[0, 1].plot(
            a["time_s"],
            np.linalg.norm(
                b["bucket_trajectory"] - second_gpu["bucket_trajectory"], axis=1
            ),
            label="GPU repeat difference",
            color="tab:red",
        )
        axes[0, 1].legend()
        for ax, title in zip(
            axes.flat,
            [
                "Bucket origin Z [m]",
                "Bucket discrepancy [m]",
                "Native bucket/grain reaction [kN]; GPU export invalid",
                "Max equality velocity residual [m/s]",
            ],
        ):
            ax.set(title=title, xlabel="Time [s]")
            ax.grid(alpha=0.3)
        axes[0, 0].legend()
        axes[1, 0].legend()
        axes[1, 1].legend()
        fig.suptitle("Full 33.3083 s identical-effort replay — MuJoCo Newton CPU/GPU")
        fig.tight_layout()
        fig.savefig(root / "full_newton.png", dpi=160)
        plt.close(fig)
        fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
        selected = [
            i
            for i, n in enumerate(a["joint_names"])
            if str(n).endswith("slider_joint") or str(n) == "articulation_joint"
        ]
        for ax, j in zip(axes.flat, selected):
            for e, s in states.items():
                ax.plot(s["time_s"], s["joint_pos"][:, j], label=e)
            ax.set_title(str(a["joint_names"][j]))
            ax.set_ylabel(
                "rad" if str(a["joint_names"][j]) == "articulation_joint" else "m"
            )
            ax.set_xlabel("Time [s]")
            ax.grid(alpha=0.3)
            ax.legend()
        fig.suptitle(
            "All five cylinder strokes and articulation — identical-effort Newton replay"
        )
        fig.tight_layout()
        fig.savefig(root / "full_newton_cylinders.png", dpi=160)
        plt.close(fig)
    feedback = root / "feedback_ground"
    if all((feedback / e / "report.json").exists() for e in ["native", "mjwarp"]):
        ar, a, aa = read(feedback / "native")
        br, b, ba = read(feedback / "mjwarp")
        aligned(a, b)
        if ar["feedback_reference_sha256"] != br["feedback_reference_sha256"]:
            raise ValueError("Feedback references differ")
        if ar["feedback_configuration"] != br["feedback_configuration"]:
            raise ValueError("Feedback gains, limits or semantics differ")
        for report, audit in [(ar, aa), (br, ba)]:
            limits = np.asarray(
                report["feedback_configuration"]["effort_limits_N_or_Nm"]
            )
            if np.any(abs(audit["effort_command"]) > limits + 1e-9):
                raise ValueError("Feedback command exceeded common limits")
        results["feedback"] = {
            "native": ar,
            "mjwarp": br,
            "bucket": trajectory_error(a["bucket_trajectory"], b["bucket_trajectory"]),
            "max_effort_difference_by_actuator": abs(
                aa["effort_command"] - ba["effort_command"]
            )
            .max(axis=0)
            .tolist(),
        }
        fig, axes = plt.subplots(2, 1, figsize=(10, 7))
        for e, s, d in [("native", a, aa), ("mjwarp", b, ba)]:
            axes[0].plot(s["time_s"], s["bucket_trajectory"][:, 2], label=e)
            axes[1].plot(
                np.r_[0, s["time_s"]], d["effort_command"][:, 0] / 1000, label=e
            )
        axes[0].set_ylabel("Bucket origin Z [m]")
        axes[1].set_ylabel("Curl effort [kN]")
        axes[1].set_xlabel("Time [s]")
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.legend()
        fig.suptitle("Separate common bounded feedback diagnostic — unloaded ground")
        fig.tight_layout()
        fig.savefig(root / "feedback.png", dpi=160)
        plt.close(fig)
    (root / "comparison.json").write_text(
        json.dumps(results, indent=2, allow_nan=False) + "\n"
    )
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--root", type=Path, default=Path("outputs/wa320/sensitivity_investigation")
    )
    evaluate(p.parse_args().root)
