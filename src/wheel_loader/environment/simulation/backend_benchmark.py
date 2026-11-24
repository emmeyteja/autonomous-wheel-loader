"""Bounded open-loop M2 replay; no terrain framework or training integration."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np

from wheel_loader.evaluation.material_accounting import (
    AcquisitionFunnel,
    MaterialAccounting,
    bucket_payload_mask,
)


def load_bundle(folder):
    import mujoco

    folder = Path(folder)
    manifest = json.loads((folder / "benchmark.json").read_text())
    for name, digest in manifest["sha256"].items():
        if hashlib.sha256((folder / name).read_bytes()).hexdigest() != digest:
            raise ValueError("Benchmark bundle checksum mismatch: " + name)
    model = mujoco.MjModel.from_binary_path(str(folder / "compiled_model.mjb"))
    with np.load(folder / "command_tape.npz") as n:
        tape = {k: n[k] for k in n.files}
    if (
        tape["effort"].shape != (manifest["control_steps"], model.nu)
        or not np.isfinite(tape["effort"]).all()
    ):
        raise ValueError("Invalid physical-effort tape")
    if not np.isclose(
        model.opt.timestep * manifest["substeps"], manifest["control_dt_s"]
    ):
        raise ValueError("Physics/control timestep mismatch")
    return model, tape, manifest


def validate_parameters(parameters, manifest):
    digest = hashlib.sha256(json.dumps(parameters, sort_keys=True).encode()).hexdigest()
    if digest != manifest["parameter_sha256"]:
        raise ValueError("Vehicle/accounting parameters differ from captured bundle")


class Recorder:
    """Shared 20 Hz ground-truth measurements. Unsupported forces are None, never zero."""

    def __init__(self, model, initial, parameters, sampling_hz=20):
        import mujoco

        self.model, self.parameters = model, parameters
        self.sampling_hz = sampling_hz
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            for b in range(model.nbody)
        ]
        self.bucket = next(i for i, n in enumerate(names) if n.endswith("_bucket"))
        self.boom = next(i for i, n in enumerate(names) if n.endswith("_boom"))
        self.rear = next(i for i, n in enumerate(names) if n.endswith("_rear_frame"))
        self.grains = np.array(
            [i for i, n in enumerate(names) if n.startswith("grain_")], dtype=int
        )
        self.masses = model.body_mass[self.grains]
        self.initial_grains = initial.xpos[self.grains].copy()
        self.source_lo = (
            self.initial_grains.min(axis=0) - 0.12 if len(self.grains) else np.zeros(3)
        )
        self.source_hi = (
            self.initial_grains.max(axis=0) + 0.12 if len(self.grains) else np.zeros(3)
        )
        self.account = MaterialAccounting(self.masses)
        self.funnel = (
            AcquisitionFunnel(self.masses, parameters) if len(self.grains) else None
        )
        self.rows = []
        self.qpos = []
        self.body_positions = []
        self.body_rotations = []
        self.phases = []
        self.solver_iterations = []
        self.gaps = []
        self.reactions = []
        self.wheel_reactions = []
        self.joints = []
        self.joint_velocities = []
        self.contact_counts = []
        self.actuator_forces = []
        self.constraint_forces = []
        self.joint_ids = np.array(
            [
                j
                for j in range(model.njnt)
                if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE
            ]
        )
        self.addresses = model.jnt_qposadr[self.joint_ids]
        self.joint_names = [
            (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) or "").split(
                "_Physics_"
            )[-1]
            for j in self.joint_ids
        ]
        self.wheels = [
            int(model.jnt_bodyid[j])
            for j, n in zip(self.joint_ids, self.joint_names)
            if n.endswith("wheel_joint")
        ]

    def sample(self, t, phase, data, contacts):
        m = self.model
        pos = data.xpos
        rot = data.xmat.reshape(m.nbody, 3, 3)
        local = (pos[self.grains] - pos[self.bucket]) @ rot[self.bucket]
        inside = bucket_payload_mask(local, self.parameters)
        contacting = np.zeros(len(self.grains), bool)
        reaction = 0.0
        wheel = np.zeros((4, 3))
        for a, b, force in contacts:
            if self.bucket in (a, b):
                other = b if a == self.bucket else a
                ix = np.flatnonzero(self.grains == other)
                if len(ix):
                    contacting[ix] = True
                    reaction += float(np.linalg.norm(force))
            if 0 in (a, b):
                for k, w in enumerate(self.wheels):
                    if w in (a, b):
                        wheel[k] += (1 if b == w else -1) * force
        if self.funnel is not None:
            self.funnel.sample(local, inside, contacting, phase, t)
        xyz = pos[self.grains]
        source = ((xyz >= self.source_lo) & (xyz <= self.source_hi)).all(axis=1)
        lost = (np.linalg.norm(xyz[:, :2], axis=1) > 50) | (xyz[:, 2] < -1)
        row = self.account.sample(inside, source, lost)
        self.rows.append(row)
        self.phases.append(phase)
        self.qpos.append(data.qpos.copy())
        self.joints.append(data.qpos[self.addresses].copy())
        self.joint_velocities.append(
            data.qvel[m.jnt_dofadr[self.joint_ids]].copy()
            if hasattr(data, "qvel")
            else np.full(len(self.joint_ids), np.nan)
        )
        self.actuator_forces.append(
            data.actuator_force.copy()
            if hasattr(data, "actuator_force")
            else np.full(m.nu, np.nan)
        )
        self.constraint_forces.append(
            data.qfrc_constraint.copy()
            if hasattr(data, "qfrc_constraint")
            else np.full(m.nv, np.nan)
        )
        self.contact_counts.append(len(contacts))
        self.body_positions.append(pos.copy())
        self.body_rotations.append(rot.copy())
        self.reactions.append(reaction)
        self.wheel_reactions.append(wheel)
        gaps = []
        for i in range(m.neq):
            a, b = m.eq_obj1id[i], m.eq_obj2id[i]
            x = pos[a] + rot[a] @ m.eq_data[i, :3]
            y = pos[b] + rot[b] @ m.eq_data[i, 3:6]
            gaps.append(np.linalg.norm(x - y))
        self.gaps.append(max(gaps, default=0.0))
        if hasattr(data, "solver_niter"):
            self.solver_iterations.append(int(np.max(data.solver_niter)))

    def save(self, output, times, backend, timing, extra):
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        positions = np.asarray(self.body_positions)
        final = positions[-1, self.grains]
        inside_mass = self.rows[-1]["bucket_mass_kg"]
        rot = np.asarray(self.body_rotations)
        local = (final - positions[-1, self.bucket]) @ rot[-1, self.bucket]
        inside = bucket_payload_mask(local, self.parameters)
        source = ((final >= self.source_lo) & (final <= self.source_hi)).all(axis=1)
        radii = np.array(
            [
                self.model.geom_size[np.flatnonzero(self.model.geom_bodyid == b)[0], 0]
                for b in self.grains
            ]
        )
        funnel = (
            self.funnel.report(
                inside,
                source,
                self.initial_grains,
                final,
                radii,
                [self.source_lo, self.source_hi],
            )
            if self.funnel is not None
            else None
        )
        if funnel is not None:
            funnel["observation"] = (
                f"{self.sampling_hz:g} Hz centre-segment aperture crossings/contact samples; unique-ID masses; breakout is first reverse observation"
            )
        profile = np.asarray(self.parameters["bucket"]["profile"])
        edge = np.array([profile[0, 0], 0, profile[0, 1]])
        lip = (
            np.einsum("nij,j->ni", rot[:, self.bucket], edge)
            + positions[:, self.bucket]
        )
        articulation = self.joint_names.index("articulation_joint")
        result = {
            "backend": backend,
            "comparison": "sim-to-sim, not real WA320 accuracy validation",
            "sampling_hz": self.sampling_hz,
            "material_present": bool(len(self.grains)),
            "completed": bool(extra.pop("completed", True)),
            "retained_material_kg": inside_mass,
            "acquisition_funnel": funnel,
            "peak_bucket_reaction_N": max(self.reactions)
            if extra.get("contact_forces_available", True)
            else None,
            "max_pin_gap_m": max(self.gaps),
            "maximum_sampled_solver_iterations": max(self.solver_iterations)
            if self.solver_iterations
            else None,
            "maximum_mass_residual_kg": max(
                abs(r["conservation_residual_kg"]) for r in self.rows
            ),
            "pile_rms_displacement_m": float(
                np.sqrt(np.mean(np.sum((final - self.initial_grains) ** 2, axis=1)))
            )
            if len(self.grains)
            else None,
            "force_semantics": "sampled sum of bucket/grain force magnitudes; wheel reactions are signed world XYZ",
            "penetration_definition": "computed from lip trajectory by evaluation script against common initial sphere envelope",
            **timing,
            **extra,
        }
        np.savez_compressed(
            output / "states.npz",
            time_s=times,
            qpos=self.qpos,
            body_positions=positions,
            body_rotations=rot,
            joint_pos=self.joints,
            joint_velocity=self.joint_velocities,
            actuator_force=self.actuator_forces,
            generalized_constraint_force=self.constraint_forces,
            contact_count=self.contact_counts,
            solver_iterations=self.solver_iterations,
            connect_gap_m=self.gaps,
            joint_names=self.joint_names,
            bucket_trajectory=positions[:, self.bucket],
            boom_trajectory=positions[:, self.boom],
            lip_trajectory=lip,
            articulation_rad=np.asarray(self.joints)[:, articulation],
            bucket_reaction_N=self.reactions
            if extra.get("contact_forces_available", True)
            else np.full(len(times), np.nan),
            wheel_ground_reaction_N=self.wheel_reactions
            if extra.get("contact_forces_available", True)
            else np.full((len(times), 4, 3), np.nan),
            grain_body_ids=self.grains,
            grain_masses_kg=self.masses,
            initial_pile=self.initial_grains,
            final_pile=final,
            accounting=np.array([list(r.values()) for r in self.rows]),
            accounting_columns=list(self.rows[0]),
            phase=self.phases,
        )
        (output / "report.json").write_text(json.dumps(result, indent=2) + "\n")
        return result


def native_contacts(model, data):
    import mujoco

    contacts = []
    for ci, c in enumerate(data.contact[: data.ncon]):
        a, b = model.geom_bodyid[c.geom]
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, ci, force)
        contacts.append((int(a), int(b), c.frame.reshape(3, 3).T @ force[:3]))
    return contacts


def run_replay(bundle, output, backend, parameters, max_ticks=None):
    import mujoco

    startup = time.perf_counter()
    model, tape, manifest = load_bundle(bundle)
    validate_parameters(parameters, manifest)
    data = mujoco.MjData(model)
    data.qpos[:] = tape["initial_qpos"]
    data.qvel[:] = tape["initial_qvel"]
    mujoco.mj_forward(model, data)
    rec = Recorder(
        model,
        data,
        parameters,
        1 / (manifest["control_dt_s"] * manifest["record_every_ticks"]),
    )
    gpu = None
    graph = None
    if backend == "mjwarp":
        import mujoco_warp as mw
        import warp as wp

        wp.init()
        gpu_model = mw.put_model(model)
        gpu = mw.put_data(model, data, nconmax=4096, njmax=16384)
        # Warm kernels on separate scratch data; the benchmark state is never advanced here.
        scratch = mw.put_data(model, data, nconmax=4096, njmax=16384)
        mw.step(gpu_model, scratch)
        wp.synchronize()
        del scratch
        before = gpu.qpos.numpy().copy()
        with wp.ScopedCapture() as capture:
            for _ in range(manifest["substeps"]):
                mw.step(gpu_model, gpu)
        graph = capture.graph
        if not np.array_equal(before, gpu.qpos.numpy()):
            raise RuntimeError("Graph preparation changed the benchmark initial state")
    startup_s = time.perf_counter() - startup
    physics_s = 0.0
    times = []
    warning_max = 0
    start = time.perf_counter()
    stop_reason = None
    count = (
        manifest["control_steps"]
        if max_ticks is None
        else min(max_ticks, manifest["control_steps"])
    )
    for tick in range(count):
        effort = tape["effort"][tick]
        if gpu is None:
            data.ctrl[:] = effort
        else:
            wp.copy(
                gpu.ctrl, wp.array(effort[None, :], dtype=wp.float32, device="cuda:0")
            )
        t = time.perf_counter()
        if gpu is None:
            for _ in range(manifest["substeps"]):
                mujoco.mj_step(model, data)
        else:
            wp.capture_launch(graph)
            wp.synchronize()
        physics_s += time.perf_counter() - t
        if tick % manifest["record_every_ticks"] == 0 or tick == count - 1:
            if gpu is None:
                mujoco.mj_forward(model, data)
            else:
                mw.forward(gpu_model, gpu)
                wp.synchronize()
                mw.get_data_into(data, model, gpu)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                stop_reason = "nonfinite_state"
                break
            rec.sample(
                (tick + 1) * manifest["control_dt_s"],
                str(tape["phase"][tick]),
                data,
                native_contacts(model, data),
            )
            times.append((tick + 1) * manifest["control_dt_s"])
            warning_max = max(warning_max, int(data.warning.number.sum()))
        if tick % 240 == 0:
            print(f"{backend}: {tick}/{count} ticks", flush=True)
    elapsed = time.perf_counter() - start
    duration = (tick + 1) * manifest["control_dt_s"]
    return rec.save(
        output,
        times,
        backend,
        {
            "startup_s": startup_s,
            "instrumented_wall_s": elapsed,
            "physics_step_wall_s": physics_s,
            "simulated_s": duration,
            "instrumented_real_time_factor": duration / elapsed,
            "physics_step_real_time_factor": duration / physics_s,
        },
        {
            "completed": stop_reason is None and count == manifest["control_steps"],
            "stop_reason": stop_reason,
            "solver_warning_count": warning_max,
            "parameter_sha256": manifest["parameter_sha256"],
            "bundle_sha256": manifest["sha256"],
            "native_model_options": {
                "substeps": manifest["substeps"],
                "solver": int(model.opt.solver),
                "iterations": int(model.opt.iterations),
                "integrator": int(model.opt.integrator),
                "tolerance": float(model.opt.tolerance),
            },
            "gpu_effective_tolerance": float(gpu_model.opt.tolerance)
            if gpu is not None
            else None,
            "gpu_final_overflow_bitmask": int(
                np.bitwise_or.reduce(gpu.overflow.numpy())
            )
            if gpu is not None
            else None,
            "capacity_check_scope": "GPU final step only; not a transient-overflow audit",
            "implementation": "native MuJoCo"
            if gpu is None
            else "direct MuJoCo-Warp; Newton USD import/coupling NOT tested",
        },
    )
