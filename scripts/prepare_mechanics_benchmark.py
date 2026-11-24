"""Derive bounded no-pile diagnostics from the preserved M2 command bundle."""

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from wheel_loader.environment.simulation.backend_benchmark import load_bundle


def prepare(
    parent,
    output,
    case,
    seconds=3.0,
    iterations=None,
    substeps=None,
    solver=None,
    tolerance=None,
    curl_velocity_perturbation=0.0,
):
    original, tape, manifest = load_bundle(parent)
    if np.any(tape["initial_qvel"]):
        raise ValueError(
            "Diagnostic currently requires the common zero-velocity release"
        )
    spec = mujoco.MjSpec.from_file(str(parent / "model.xml"))
    for body in list(spec.worldbody.bodies):
        if body.name.startswith("grain_") and case != "pile_reference":
            spec.delete(
                body
            )  # MuJoCo also removes the referencing grain contact pairs.
    if case == "fixed_no_contact":
        spec.delete(
            spec.joints[0]
        )  # Mount the rear chassis; all 19 mechanism joints remain.
    if case in {"fixed_no_contact", "floating_no_contact"}:
        spec.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    compiled = spec.compile()
    model = (
        mujoco.MjModel.from_binary_path(str(parent / "compiled_model.mjb"))
        if case == "pile_reference"
        else compiled
    )
    initial = mujoco.MjData(model)
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        old = mujoco.mj_name2id(original, mujoco.mjtObj.mjOBJ_JOINT, name)
        n = 7 if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE else 1
        initial.qpos[model.jnt_qposadr[j] : model.jnt_qposadr[j] + n] = tape[
            "initial_qpos"
        ][original.jnt_qposadr[old] : original.jnt_qposadr[old] + n]
    # XML serializes some rotations with reduced precision. Restore exact collider
    # rotations from the authoritative compiled parent before saving the new MJB.
    for g in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        old = mujoco.mj_name2id(original, mujoco.mjtObj.mjOBJ_GEOM, name)
        if old < 0:
            raise ValueError("Unmapped collider in diagnostic")
        model.geom_quat[g] = original.geom_quat[old]
        spec.geom(name).quat = original.geom_quat[old]
    audit = {}
    for field in (
        "body_mass",
        "body_inertia",
        "body_pos",
        "body_quat",
        "body_ipos",
        "body_iquat",
        "geom_pos",
        "geom_size",
        "geom_quat",
        "eq_data",
        "eq_solref",
        "eq_solimp",
        "actuator_gear",
        "actuator_gainprm",
        "actuator_biasprm",
        "geom_friction",
        "geom_solref",
        "geom_solimp",
        "geom_contype",
        "geom_conaffinity",
        "pair_friction",
        "pair_solref",
        "pair_solimp",
    ):
        a, b = getattr(original, field), getattr(model, field)
        audit[field] = float(np.max(abs(a[: len(b)] - b), initial=0))
    if any(v != 0 for v in audit.values()):
        raise ValueError("Physical parameter round-trip mismatch: " + str(audit))
    steps = round(seconds / manifest["control_dt_s"])
    if not 0 < steps <= manifest["control_steps"]:
        raise ValueError("Diagnostic exceeds available effort tape")
    if iterations is not None:
        if iterations <= 0:
            raise ValueError("Iterations must be positive")
        model.opt.iterations = iterations
    if solver is not None:
        model.opt.solver = {
            "CG": mujoco.mjtSolver.mjSOL_CG,
            "Newton": mujoco.mjtSolver.mjSOL_NEWTON,
        }[solver]
    if tolerance is not None:
        if not np.isfinite(tolerance) or tolerance <= 0:
            raise ValueError("Tolerance must be finite and positive")
        model.opt.tolerance = tolerance
    chosen_substeps = manifest["substeps"] if substeps is None else substeps
    if chosen_substeps <= 0:
        raise ValueError("Substeps must be positive")
    model.opt.timestep = manifest["control_dt_s"] / chosen_substeps
    release_qpos = initial.qpos.copy()
    mujoco.mj_setConst(model, initial)
    initial.qpos[:] = release_qpos  # mj_setConst uses/resets the reference pose.
    mujoco.mj_forward(model, initial)
    reference = mujoco.MjData(original)
    reference.qpos[:] = tape["initial_qpos"]
    mujoco.mj_forward(original, reference)
    pose_error = float(
        np.linalg.norm(initial.xpos - reference.xpos[: model.nbody], axis=1).max()
    )
    if pose_error > 1e-9:
        raise ValueError(
            "Vehicle release pose changed in preparation: " + str(pose_error)
        )
    if not np.isfinite(curl_velocity_perturbation):
        raise ValueError("Velocity perturbation must be finite")
    if curl_velocity_perturbation:
        joint = next(
            j
            for j in range(model.njnt)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or "").endswith(
                "bucket_slider_joint"
            )
        )
        initial.qvel[model.jnt_dofadr[joint]] += curl_velocity_perturbation
    output.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveModel(model, str(output / "compiled_model.mjb"), None)
    spec.option.timestep = model.opt.timestep
    spec.option.iterations = model.opt.iterations
    spec.option.solver = model.opt.solver
    spec.option.tolerance = model.opt.tolerance
    spec.to_file(str(output / "model.xml"))
    np.savez_compressed(
        output / "command_tape.npz",
        initial_qpos=initial.qpos,
        initial_qvel=initial.qvel,
        effort=tape["effort"][:steps],
        phase=tape["phase"][:steps],
        control_dt_s=manifest["control_dt_s"],
    )
    result = {
        **manifest,
        "scenario": case,
        "control_steps": steps,
        "duration_s": steps * manifest["control_dt_s"],
        "substeps": chosen_substeps,
        "record_every_ticks": 1,
        "diagnostic": {
            "case": case,
            "parent_bundle_sha256": manifest["sha256"],
            "vehicle_parameter_max_abs_errors": audit,
            "iterations": int(model.opt.iterations),
            "solver": int(model.opt.solver),
            "tolerance": float(model.opt.tolerance),
            "initial_curl_velocity_perturbation_m_s": curl_velocity_perturbation,
            "particle_count": sum(b.name.startswith("grain_") for b in spec.bodies),
            "initial_vehicle_position_error_m": pose_error,
            "purpose": "controlled solver/isolation study; same physical effort tape",
        },
        "sha256": {
            name: hashlib.sha256((output / name).read_bytes()).hexdigest()
            for name in manifest["sha256"]
        },
    }
    (output / "benchmark.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--parent", type=Path, default=Path("outputs/wa320/backend_benchmark/bundle")
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--case",
        choices=[
            "fixed_no_contact",
            "floating_no_contact",
            "unloaded_ground",
            "pile_reference",
        ],
        required=True,
    )
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--iterations", type=int)
    parser.add_argument("--substeps", type=int)
    parser.add_argument("--solver", choices=["CG", "Newton"])
    parser.add_argument("--tolerance", type=float)
    parser.add_argument("--curl-velocity-perturbation", type=float, default=0.0)
    args = parser.parse_args()
    prepare(
        args.parent,
        args.output,
        args.case,
        args.seconds,
        args.iterations,
        args.substeps,
        args.solver,
        args.tolerance,
        args.curl_velocity_perturbation,
    )
