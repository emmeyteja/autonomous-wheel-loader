"""Constraint diagnostics for the preserved WA320 effort-replay experiment."""

import numpy as np


def equality_jacobian(model, data):
    import mujoco

    if mujoco.mj_isSparse(model):
        jac = np.zeros((data.nefc, model.nv))
        mujoco.mju_sparse2dense(
            jac, data.efc_J, data.efc_J_rownnz, data.efc_J_rowadr, data.efc_J_colind
        )
    else:
        jac = data.efc_J.reshape(data.nefc, model.nv)
    return jac[: data.ne].copy()


def consistent_bucket_velocity(model, data, speed):
    """Solve J v=0 with root/wheels/grains locked and four other cylinders held.

    Minimum Euclidean norm in the mechanism's mixed SI joint coordinates; no
    claim of a mass-weighted projection. Bucket cylinder speed is an exact row.
    """
    import mujoco

    names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
        for j in range(model.njnt)
    ]
    joints = [
        j
        for j, n in enumerate(names)
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE
        and not n.endswith("wheel_joint")
    ]
    dofs = model.jnt_dofadr[joints]
    jac = equality_jacobian(model, data)[:, dofs]
    sliders = [k for k, j in enumerate(joints) if names[j].endswith("slider_joint")]
    prescribed = np.zeros((len(sliders), len(dofs)))
    target = np.zeros(len(sliders))
    for row, col in enumerate(sliders):
        prescribed[row, col] = 1
        if names[joints[col]].endswith("bucket_slider_joint"):
            target[row] = speed
    a = np.vstack([jac, prescribed])
    rhs = np.r_[np.zeros(len(jac)), target]
    velocity, _, rank, singular = np.linalg.lstsq(a, rhs, rcond=1e-12)
    if np.max(abs(a @ velocity - rhs)) > max(abs(speed) * 1e-7, 1e-16):
        raise ValueError(
            "Requested cylinder tangent cannot satisfy linkage constraints"
        )
    result = np.zeros(model.nv)
    result[dofs] = velocity
    return result, {
        "solve_rank": int(rank),
        "singular_values": singular.tolist(),
        "max_equation_residual": float(np.max(abs(a @ velocity - rhs))),
        "mechanism_dof_names": [names[j] for j in joints],
        "mechanism_velocity": velocity.tolist(),
    }


class MechanismAudit:
    def __init__(self, model):
        self.model = model
        self.rows = []

    def sample(self, data, effort):
        import mujoco

        jac = equality_jacobian(self.model, data)
        # get_data_into copies M and qvel, so energy uses the backend's mass matrix.
        mujoco.mj_energyPos(self.model, data)
        mujoco.mj_energyVel(self.model, data)
        singular = np.linalg.svd(jac[:, : min(self.model.nv, 25)], compute_uv=False)
        self.rows.append(
            {
                "equality_position": data.efc_pos[: data.ne].copy(),
                "equality_velocity": jac @ data.qvel,
                "equality_force": data.efc_force[: data.ne].copy(),
                "generalized_constraint_force": data.qfrc_constraint.copy(),
                "energy_potential_kinetic_J": data.energy.copy(),
                "effort_command": np.asarray(effort).copy(),
                "equality_singular_values": singular,
                "warning_number": data.warning.number.copy(),
            }
        )

    def save(self, path, overflow):
        np.savez_compressed(
            path,
            **{key: np.asarray([r[key] for r in self.rows]) for key in self.rows[0]},
            gpu_overflow_by_tick=np.asarray(overflow, dtype=np.int64),
        )
