"""World-frame diagnostics reconstructed from recorded rigid-body configurations.

No physics is advanced. Velocities are central differences in simulation time,
not playback time; cylinder endpoints are estimated visual mounting points.
"""

import numpy as np


def world_angular_velocity(before, after, dt):
    """SO(3) log of R_after R_before.T / dt, expressed in world axes."""
    relative = after @ before.T
    cosine = np.clip((np.trace(relative) - 1) / 2, -1, 1)
    angle = np.arccos(cosine)
    skew = np.array(
        [
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        ]
    )
    if angle < 1e-7:
        return skew / (2 * dt)
    if np.pi - angle < 1e-5:
        raise ValueError("Telemetry rotation difference too close to pi")
    return skew * angle / (2 * np.sin(angle) * dt)


def rpy_degrees(rotation):
    """World orientation: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    return np.degrees(
        [
            np.arctan2(rotation[2, 1], rotation[2, 2]),
            np.arcsin(np.clip(-rotation[2, 0], -1, 1)),
            np.arctan2(rotation[1, 0], rotation[0, 0]),
        ]
    )


class ReplayState:
    def __init__(self, model, names, positions, roots, times, parameters):
        import mujoco

        self.mj, self.model, self.p = mujoco, model, parameters
        self.positions, self.roots, self.times = positions, roots, times
        self.names = names
        self.data = mujoco.MjData(model)
        ids = {
            (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or "").split(
                "_Physics_"
            )[-1]: i
            for i in range(model.njnt)
        }
        self.addresses = [model.jnt_qposadr[ids[name]] for name in names]
        free = np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
        if len(free) != 1:
            raise ValueError("Replay requires exactly one floating root")
        self.base = model.jnt_qposadr[free[0]]
        self.bodies = {}
        for suffix in (
            "rear_frame",
            "front_frame",
            "boom",
            "bellcrank",
            "bucket",
            "rod",
        ):
            matches = [
                i
                for i in range(model.nbody)
                if (
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
                ).endswith("_" + suffix)
            ]
            if len(matches) != 1:
                raise ValueError(f"Cannot resolve body {suffix}")
            self.bodies[suffix] = matches[0]
        self.initial_lengths = self.configuration(0)["lengths"]

    def configuration(self, index):
        data, root = self.data, self.roots[index]
        data.qpos[self.addresses] = self.positions[index]
        data.qpos[self.base : self.base + 3] = root[:3]
        data.qpos[self.base + 3 : self.base + 7] = root[[6, 3, 4, 5]]
        self.mj.mj_forward(self.model, data)

        def point(body, local):
            i = self.bodies[body]
            return data.xpos[i] + data.xmat[i].reshape(3, 3) @ local

        frames = {}
        for label, body, local in (
            ("CABIN (cab centre)", "rear_frame", self.p["geometry"]["cab_center"]),
            ("HEAD (front hinge)", "front_frame", [0, 0, 0]),
            ("BUCKET (pivot)", "bucket", [0, 0, 0]),
        ):
            frames[label] = (
                point(body, local),
                data.xmat[self.bodies[body]].reshape(3, 3).copy(),
            )
        b, k = self.p["boom"], self.p["linkage"]
        # Visual-only estimates: no manufacturer mounting drawings available.
        pins = {}
        for label, sign in (("Lift L", 1), ("Lift R", -1)):
            base = np.array(self.p["geometry"]["lift_cylinder_base"], dtype=float)
            rod = np.array(self.p["geometry"]["lift_cylinder_rod_pin"], dtype=float)
            base[1] *= sign
            rod[1] *= sign
            pins[label] = (point("front_frame", base), point("boom", rod))
        pins["Tilt"] = (
            point(
                "front_frame",
                [
                    b["pivot"][0] + k["tilt_cylinder_base"][0],
                    0,
                    b["pivot"][1] + k["tilt_cylinder_base"][1],
                ],
            ),
            point("bellcrank", [k["bellcrank_upper_length"], 0, 0]),
        )
        return {
            "frames": frames,
            "pins": pins,
            "lengths": {
                name: float(np.linalg.norm(tip - base))
                for name, (base, tip) in pins.items()
            },
            "pin_axis": data.xmat[self.bodies["front_frame"]]
            .reshape(3, 3)[:, 1]
            .copy(),
        }

    def sample(self, index):
        lo, hi = max(0, index - 1), min(len(self.times) - 1, index + 1)
        dt = self.times[hi] - self.times[lo]
        before, after = self.configuration(lo), self.configuration(hi)
        current = self.configuration(index)  # Restore the frame being rendered.
        current["velocities"] = {
            name: (
                (after["frames"][name][0] - pose[0]) / dt,
                world_angular_velocity(pose[1], after["frames"][name][1], dt),
            )
            for name, pose in before["frames"].items()
        }
        current["length_rates"] = {
            name: (after["lengths"][name] - length) / dt
            for name, length in before["lengths"].items()
        }
        current["length_changes"] = {
            name: length - self.initial_lengths[name]
            for name, length in current["lengths"].items()
        }
        current["joint_rates"] = (self.positions[hi] - self.positions[lo]) / dt
        return current
