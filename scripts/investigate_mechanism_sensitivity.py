"""Prepare constraint-tangent perturbations or run audited preserved-bundle replays."""

import argparse
import hashlib
import json
import runpy
from pathlib import Path

import mujoco
import numpy as np

from wheel_loader.environment.simulation.backend_benchmark import (
    load_bundle,
    run_replay,
)
from wheel_loader.environment.simulation.mechanism_investigation import (
    consistent_bucket_velocity,
    equality_jacobian,
)
from wheel_loader.vehicle.parameters import load_parameters


def prepare(args):
    output = args.output
    runpy.run_path(str(Path(__file__).with_name("prepare_mechanics_benchmark.py")))[
        "prepare"
    ](args.parent, output, args.case, args.seconds, solver=args.solver)
    model, tape, manifest = load_bundle(output)
    data = mujoco.MjData(model)
    data.qpos[:] = tape["initial_qpos"]
    mujoco.mj_forward(model, data)
    isolated = np.zeros(model.nv)
    bucket = next(
        j
        for j in range(model.njnt)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or "").endswith(
            "bucket_slider_joint"
        )
    )
    isolated[model.jnt_dofadr[bucket]] = args.speed
    consistent, projection = consistent_bucket_velocity(model, data, args.speed)
    initial = {}
    for name, velocity in [
        ("baseline", np.zeros(model.nv)),
        ("isolated", isolated),
        ("consistent", consistent),
    ]:
        data.qvel[:] = velocity
        data.ctrl[:] = tape["effort"][0]
        mujoco.mj_forward(model, data)
        jac = equality_jacobian(model, data)
        initial[name] = {
            "qvel": velocity.tolist(),
            "equality_position_m": data.efc_pos[: data.ne].tolist(),
            "equality_velocity_m_s": (jac @ velocity).tolist(),
            "generalized_constraint_force": data.qfrc_constraint.tolist(),
            "equality_force_N": data.efc_force[: data.ne].tolist(),
            "equality_jacobian_singular_values": np.linalg.svd(
                jac, compute_uv=False
            ).tolist(),
        }
    tape["initial_qvel"] = {
        "baseline": np.zeros(model.nv),
        "isolated": isolated,
        "consistent": consistent,
    }[args.perturbation]
    if args.zero_effort:
        tape["effort"] = np.zeros_like(tape["effort"])
    np.savez_compressed(output / "command_tape.npz", **tape)
    manifest["sha256"]["command_tape.npz"] = hashlib.sha256(
        (output / "command_tape.npz").read_bytes()
    ).hexdigest()
    manifest["investigation"] = {
        "perturbation": args.perturbation,
        "zero_effort_ablation": args.zero_effort,
        "bucket_speed_m_s": args.speed,
        "projection": projection,
        "initial_states": initial,
        "isolated_consistent_velocity_difference_norm": float(
            np.linalg.norm(isolated - consistent)
        ),
        "constraint_names": [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
            for i in range(model.neq)
        ],
    }
    (output / "benchmark.json").write_text(json.dumps(manifest, indent=2) + "\n")


def feedback_controller(model, tape, reference, parameters):
    from wheel_loader.control.trajectory_feedback import BoundedTrajectoryFeedback

    states = np.load(reference)
    names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) or ""
        for j in model.actuator_trnid[:, 0]
    ]
    qadr = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    vadr = model.jnt_dofadr[model.actuator_trnid[:, 0]]
    if len(states["qpos"]) != len(tape["effort"]) or not np.allclose(
        states["time_s"],
        (np.arange(len(tape["effort"])) + 1) * float(tape["control_dt_s"]),
        rtol=0,
        atol=1e-10,
    ):
        raise ValueError("Reference must cover every control tick")
    qref = np.vstack([tape["initial_qpos"][qadr], states["qpos"][:, qadr]])[:-1]
    # Saved joint velocity arrays contain scalar joints in model order.
    scalar_joints = np.flatnonzero(model.jnt_type != mujoco.mjtJoint.mjJNT_FREE)
    cols = [
        int(np.flatnonzero(scalar_joints == j)[0]) for j in model.actuator_trnid[:, 0]
    ]
    vref = np.vstack([tape["initial_qvel"][vadr], states["joint_velocity"][:, cols]])[
        :-1
    ]
    kp, kd, limits = [], [], []
    for name in names:
        if name.endswith("wheel_joint"):
            kp.append(0.0)
            kd.append(parameters["solver"]["wheel_damping"])
            limits.append(parameters["wheels"]["max_torque"])
        else:
            kind = (
                "bucket"
                if "bucket_slider" in name
                else "lift"
                if "lift_" in name
                else "steering"
            )
            cfg = parameters["actuators"][kind]
            kp.append(cfg["stiffness"])
            kd.append(cfg["damping"])
            limits.append(cfg["fmax"])
    controller = BoundedTrajectoryFeedback(qref, vref, tape["effort"], kp, kd, limits)

    def feedback(tick, data):
        return controller.effort(tick, data.qpos[qadr], data.qvel[vadr])

    feedback.specification = {
        "actuator_joint_names": names,
        "position_gains_N_per_m_or_Nm_per_rad": kp,
        "velocity_gains_Ns_per_m_or_Nms_per_rad": kd,
        "effort_limits_N_or_Nm": limits,
        "control_dt_s": float(tape["control_dt_s"]),
        "reference_timing": "beginning of control tick",
        "feedback": "scalar actuator-joint qpos and qvel; wheel position gain zero",
        "feedforward": "original physical-effort tape",
    }
    return feedback


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument(
        "--parent", type=Path, default=Path("outputs/wa320/backend_benchmark/bundle")
    )
    prep.add_argument(
        "--case",
        choices=[
            "fixed_no_contact",
            "floating_no_contact",
            "unloaded_ground",
            "pile_reference",
        ],
        required=True,
    )
    prep.add_argument("--seconds", type=float, default=10)
    prep.add_argument("--solver", choices=["CG", "Newton"], default="CG")
    prep.add_argument(
        "--perturbation",
        choices=["baseline", "isolated", "consistent"],
        default="baseline",
    )
    prep.add_argument("--speed", type=float, default=1e-7)
    prep.add_argument(
        "--zero-effort",
        action="store_true",
        help="Separately labelled actuator-off ablation; not the original tape comparison",
    )
    prep.add_argument("--output", type=Path, required=True)
    run = sub.add_parser("run")
    run.add_argument("--bundle", type=Path, required=True)
    run.add_argument("--backend", choices=["native", "mjwarp"], required=True)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--feedback-reference", type=Path)
    a = p.parse_args()
    if a.command == "prepare":
        prepare(a)
    else:
        parameters = load_parameters("configs/vehicle/wa320.yaml")
        feedback = None
        if a.feedback_reference:
            model, tape, _ = load_bundle(a.bundle)
            feedback = feedback_controller(
                model, tape, a.feedback_reference, parameters
            )
        report = run_replay(
            a.bundle, a.output, a.backend, parameters, audit=True, feedback=feedback
        )
        if a.feedback_reference:
            report["feedback_reference_sha256"] = hashlib.sha256(
                a.feedback_reference.read_bytes()
            ).hexdigest()
            report["feedback_definition"] = (
                "original tape feedforward + bounded PD; native Newton reference at beginning of tick; existing gains/force caps; scalar joint q/qvel SI"
            )
            report["feedback_configuration"] = feedback.specification
            (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        if not report["completed"]:
            raise SystemExit("Incomplete investigation")
