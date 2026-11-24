"""Live bucket/pile geometry -> constrained cylinder-velocity commands and finite forces."""

import itertools

import numpy as np

from wheel_loader.control.cylinder_actuation import FirstOrderCylinders, joint_efforts
from wheel_loader.control.spatial_scoop import SpatialScoop
from wheel_loader.evaluation.material_accounting import bucket_payload_mask

from .mechanism_investigation import equality_jacobian


class SpatialScoopFeedback:
    def __init__(self, model, parameters, cfg):
        import mujoco

        self.model = model
        self.p = parameters
        self.cfg = cfg
        self.controller = SpatialScoop(cfg)
        self.contacts = []
        self.log = []
        self.entry_x = None
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            for b in range(model.nbody)
        ]
        self.bucket = next(b for b, n in enumerate(names) if n.endswith("_bucket"))
        self.rear = next(b for b, n in enumerate(names) if n.endswith("_rear_frame"))
        self.grains = np.array(
            [b for b, n in enumerate(names) if n.startswith("grain_")], dtype=int
        )
        self.radii = np.array(
            [
                model.geom_size[np.flatnonzero(model.geom_bodyid == b)[0], 0]
                for b in self.grains
            ]
        )
        self.masses = model.body_mass[self.grains]
        self.joints = np.flatnonzero(model.jnt_type != mujoco.mjtJoint.mjJNT_FREE)
        self.names = [
            (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) or "").split(
                "_Physics_"
            )[-1]
            for j in self.joints
        ]
        self.qa = model.jnt_qposadr[self.joints]
        self.va = model.jnt_dofadr[self.joints]
        self.actuator_indices = [
            self.names.index(
                (
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) or ""
                ).split("_Physics_")[-1]
            )
            for j in model.actuator_trnid[:, 0]
        ]
        self.mechanism = [
            i for i, n in enumerate(self.names) if not n.endswith("wheel_joint")
        ]
        self.dofs = self.va[self.mechanism]
        self.sliders = [
            i for i, n in enumerate(self.names) if n.endswith("slider_joint")
        ]
        self.actuator = FirstOrderCylinders(
            parameters, [0, *parameters["maneuver"]["settle"]]
        )
        self.vertices = []
        for g in np.flatnonzero(model.geom_bodyid == self.bucket):
            if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                vertices = (
                    np.array(list(itertools.product([-1, 1], repeat=3)))
                    * model.geom_size[g]
                )
            elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mesh = model.geom_dataid[g]
                start = model.mesh_vertadr[mesh]
                vertices = model.mesh_vert[start : start + model.mesh_vertnum[mesh]]
            else:
                raise ValueError("Unsupported live bucket collider")
            r = np.zeros(9)
            mujoco.mju_quat2Mat(r, model.geom_quat[g])
            self.vertices.extend(vertices @ r.reshape(3, 3).T + model.geom_pos[g])
        self.vertices = np.asarray(self.vertices)
        lip = parameters["bucket"]["profile"][0]
        self.lip = np.array([lip[0], 0, lip[1]])
        edge = (
            np.array(parameters["bucket"]["profile"][0])
            - parameters["bucket"]["profile"][1]
        )
        self.edge_angle = np.arctan2(edge[1], edge[0])
        self.initialized = False
        self.drive_integral = 0.0
        self.root_velocity_address = model.jnt_dofadr[
            next(
                j
                for j in range(model.njnt)
                if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE
                and model.jnt_bodyid[j] == self.rear
            )
        ]

    @property
    def phase(self):
        return self.controller.phase

    @property
    def done(self):
        return self.controller.done or self.controller.failed

    def __call__(self, tick, data):
        import mujoco

        m = self.model
        c = self.cfg
        dt = 1 / 240
        t = tick * dt
        r = data.xmat[self.bucket].reshape(3, 3)
        pos = data.xpos[self.bucket]
        world = self.vertices @ r.T + pos
        minimum = world[np.argmin(world[:, 2])]
        lip = r @ self.lip + pos
        axis = r @ np.array([np.cos(self.edge_angle), 0, np.sin(self.edge_angle)])
        pitch = np.arctan2(axis[2], axis[0])
        jp = np.zeros((3, m.nv))
        jr = np.zeros_like(jp)
        mujoco.mj_jac(m, data, jp, jr, lip, self.bucket)
        low_jp = np.zeros_like(jp)
        low_jr = np.zeros_like(jp)
        mujoco.mj_jac(m, data, low_jp, low_jr, minimum, self.bucket)
        xyz = data.xpos[self.grains]
        local = (xyz - pos) @ r
        inside = bucket_payload_mask(local, self.p)
        mass = float(self.masses[inside].sum())
        material_forces = [
            np.linalg.norm(f)
            for a, b, f in self.contacts
            if (a == self.bucket and b in self.grains)
            or (b == self.bucket and a in self.grains)
        ]
        reaction = float(sum(material_forces))
        active = reaction > 1e-3
        pile_top = float((xyz[:, 2] + self.radii).max()) if len(xyz) else 0.0
        if self.entry_x is None and t >= c["settle_s"] and len(xyz):
            intersect = (abs(xyz[:, 2] - c["attack_height_m"]) <= self.radii) & (
                abs(xyz[:, 1] - lip[1]) < self.p["vehicle"]["bucket_width"] / 2
            )
            if not intersect.any():
                self.controller.failed = True
                self.controller.reason = "no_pile_intersection_at_attack_height"
            else:
                self.entry_x = float(
                    (
                        xyz[intersect, 0]
                        - np.sqrt(
                            np.maximum(
                                self.radii[intersect] ** 2
                                - (xyz[intersect, 2] - c["attack_height_m"]) ** 2,
                                0,
                            )
                        )
                    ).min()
                )
        o = {
            "lip_z": float(lip[2]),
            "pitch": float(pitch),
            "clearance": float(minimum[2]),
            "clearance_rate": float(low_jp[2] @ data.qvel),
            "distance_to_entry": self.entry_x - lip[0]
            if self.entry_x is not None
            else float("inf"),
            "penetration": lip[0] - self.entry_x if self.entry_x is not None else 0.0,
            "mass": mass,
            "active_material_contact": active,
            "pile_top": pile_top,
            "rear_x": float(data.xpos[self.rear, 0]),
            "reaction_force_N": reaction,
            "forward_speed_m_s": float(data.qvel[self.root_velocity_address]),
        }
        goal = self.controller.goal(t, o)
        # Three-dimensional linkage tangent, with five prescribed cylinder rates.
        jac = equality_jacobian(m, data)[:, self.dofs]
        hold = np.zeros((len(self.sliders), len(self.dofs)))
        rhs = np.zeros((len(self.sliders), 2))
        for row, i in enumerate(self.sliders):
            hold[row, self.mechanism.index(i)] = 1
            if self.names[i].startswith("lift_"):
                rhs[row, 0] = 1
            elif self.names[i] == "bucket_slider_joint":
                rhs[row, 1] = 1
        tangent = np.linalg.lstsq(
            np.vstack([jac, hold]),
            np.vstack([np.zeros((len(jac), 2)), rhs]),
            rcond=1e-10,
        )[0]
        angular_deriv = np.stack(
            [np.cross(np.eye(3)[i], axis) for i in range(3)], axis=1
        )
        pitch_gradient = (axis[0] * angular_deriv[2] - axis[2] * angular_deriv[0]) / (
            axis[0] ** 2 + axis[2] ** 2
        )
        task = np.stack([jp[2, self.dofs], pitch_gradient @ jr[:, self.dofs]]) @ tangent
        if not np.isfinite(task).all() or np.linalg.cond(task) > 1e4:
            raise RuntimeError("Ill-conditioned bucket/cylinder task Jacobian")
        desired = np.array(
            [
                c["height_gain_per_s"] * (goal[0] - lip[2]),
                c["pitch_gain_per_s"] * (goal[1] - pitch),
            ]
        )
        # Preserve the task-space direction: independent clipping can ask for
        # rotation faster than the boom can clear the ground/mechanical stops.
        desired /= max(
            1.0,
            float(
                np.max(
                    abs(desired) / [c["max_lip_speed_m_s"], c["max_pitch_speed_rad_s"]]
                )
            ),
        )
        rate = np.linalg.solve(task, desired)
        if self.phase == "curl":
            # Curl sweeps the edge upward about its pivot. Holding lip Z here
            # would command boom lowering into the mechanical/ground stops.
            pitch_rate = np.clip(
                c["pitch_gain_per_s"] * (goal[1] - pitch),
                -c["max_pitch_speed_rad_s"],
                c["max_pitch_speed_rad_s"],
            )
            rate = np.array([0.0, pitch_rate / task[1, 1]])
        # Project the commanded cylinder rate onto a one-step clearance inequality.
        gradient = low_jp[2, self.dofs] @ tangent
        required = min(
            c["max_lip_speed_m_s"],
            max(-c["max_lip_speed_m_s"], 3 * (c["minimum_clearance_m"] - minimum[2])),
        )
        if gradient @ rate < required:
            rate += (
                gradient
                * (required - gradient @ rate)
                / max(gradient @ gradient, 1e-12)
            )
        caps = np.array([self.p["actuators"][n]["vmax"] for n in ["lift", "bucket"]])
        rate /= max(1.0, float(np.max(abs(rate) / caps)))
        for _ in range(3):
            for col, i in enumerate(self.mechanism):
                j = self.joints[i]
                if not m.jnt_limited[j]:
                    continue
                row = tangent[col]
                lo, hi = (m.jnt_range[j] - data.qpos[self.qa[i]]) / c[
                    "braking_horizon_s"
                ]
                value = row @ rate
                correction = np.clip(value, lo, hi) - value
                if correction:
                    rate += row * correction / max(row @ row, 1e-12)
            if gradient @ rate < required:
                rate += (
                    gradient
                    * (required - gradient @ rate)
                    / max(gradient @ gradient, 1e-12)
                )
            rate /= max(1.0, float(np.max(abs(rate) / caps)))
        drive_error = goal[2] - o["forward_speed_m_s"]
        if goal[2] == 0:
            self.drive_integral = 0.0
        candidate = self.drive_integral + c["drive_integral_gain"] * drive_error * dt
        drive_reference = goal[2] + c["drive_speed_gain"] * drive_error + candidate
        drive_cap = self.p["actuators"]["drive"]["vmax"]
        if abs(drive_reference) < drive_cap or drive_reference * drive_error <= 0:
            self.drive_integral = candidate
        u = np.array(
            [
                np.clip(drive_reference / drive_cap, -1, 1),
                0,
                rate[0] / caps[0],
                rate[1] / caps[1],
            ]
        )
        q = data.qpos[self.qa]
        v = data.qvel[self.va]
        if not self.initialized:
            self.actuator.x[0] = [
                q[self.names.index(n + "_slider_joint")]
                + self.p["cylinder_bodies"][n]["reference_length"]
                for n in ["steering_left", "lift_left", "bucket"]
            ]
            self.initialized = True
        efforts, _ = joint_efforts(self.p, self.actuator, u, q, v, self.names, dt)
        self.log.append(
            [
                t,
                lip[0],
                lip[2],
                pitch,
                minimum[2],
                o["penetration"],
                mass,
                float(active),
                goal[0],
                goal[1],
                goal[2],
                rate[0],
                rate[1],
                reaction,
            ]
        )
        return efforts[self.actuator_indices]
