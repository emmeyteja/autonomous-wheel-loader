"""Bounded first-order actuator proxy and virtual-work force transmission.

Normalized [drive, steering, lift, bucket] commands. No simulator/ROS imports.
Cylinder reference stroke is integrated; actual stroke comes from linkage sensors.
Finite servo forces can stall: no body/joint state is overwritten.
"""

import numpy as np

from wheel_loader.vehicle.kinematics import rotate

NAMES = ("steering", "lift", "bucket")


def cylinder_geometry(p, angles):
    """Lengths [3] and d(length)/d(articulation,boom,tilt) [3,3].

    Steering uses the left cylinder; equal/opposite right force is included in
    torque transmission. Lift has two identical cylinders. All mounts estimated.
    """
    a, b, t = angles
    k = p["linkage"]
    boom = p["boom"]
    g = p["geometry"]
    q = boom["initial_elevation"] + b
    phi = k["initial_bellcrank_angle"] + t
    base = np.array(g["steering_base"][:2])
    front = np.array(g["steering_front_pin"][:2])
    tip = rotate(a, front)
    delta = tip - base
    ls = np.linalg.norm(delta)
    js = delta @ np.array([-tip[1], tip[0]]) / ls
    front_r = front * [1, -1]
    base_r = base * [1, -1]
    tip_r = rotate(a, front_r)
    dr = tip_r - base_r
    jsr = dr @ np.array([-tip_r[1], tip_r[0]]) / np.linalg.norm(dr)
    pin = np.array(g["lift_cylinder_rod_pin"])[[0, 2]]
    rp = rotate(q, pin)
    dl = np.array(boom["pivot"]) + rp - np.array(g["lift_cylinder_base"])[[0, 2]]
    ll = np.linalg.norm(dl)
    jl = dl @ np.array([-rp[1], rp[0]]) / ll
    upper = rotate(phi, [k["bellcrank_upper_length"], 0])
    c = np.array(k["bellcrank_pivot"])
    ep = rotate(q, c + upper)
    db = ep - np.array(k["tilt_cylinder_base"])
    lb = np.linalg.norm(db)
    jb = db @ np.array([-ep[1], ep[0]]) / lb
    ephi = rotate(q, upper)
    jt = db @ np.array([-ephi[1], ephi[0]]) / lb
    jac = np.array([[js, 0, 0], [0, jl, 0], [0, jb, jt]])
    transmission = jac.copy()
    transmission[0, 0] = js - jsr
    transmission[1] *= 2
    return np.array([ls, ll, lb]), jac, transmission


class FirstOrderCylinders:
    def __init__(self, p, initial_angles, count=1):
        self.p = p
        self.lo = np.array([p["cylinders"][n]["pin_distance_min"] for n in NAMES])
        self.hi = np.array([p["cylinders"][n]["pin_distance_max"] for n in NAMES])
        self.tau = np.array([p["actuators"][n]["tau"] for n in NAMES])
        self.vmax = np.array([p["actuators"][n]["vmax"] for n in NAMES])
        self.fmax = np.array([p["actuators"][n]["fmax"] for n in NAMES])
        self.kp = np.array([p["actuators"][n]["stiffness"] for n in NAMES])
        self.ki = np.array([p["actuators"][n]["integral_gain"] for n in NAMES])
        self.integral_force = np.zeros((count, 3))
        self.kd = np.array([p["actuators"][n]["damping"] for n in NAMES])
        if not all(
            np.isfinite(v).all() and (v > 0).all()
            for v in (self.tau, self.vmax, self.fmax, self.kp, self.ki, self.kd)
        ) or np.any(self.hi <= self.lo):
            raise ValueError("Invalid first-order cylinder parameters")
        lengths = cylinder_geometry(p, initial_angles)[0]
        self.initial = np.tile(lengths, (count, 1))
        self.x = self.initial.copy()
        self.v = np.zeros_like(self.x)
        self.drive = np.zeros(count)

    def reset(self, mask):
        mask = np.asarray(mask, bool)
        if mask.shape != (len(self.x),):
            raise ValueError("Invalid reset mask")
        self.x[mask] = self.initial[mask]
        self.v[mask] = 0
        self.drive[mask] = 0
        self.integral_force[mask] = 0

    def step(self, commands, dt, measured_lengths, measured_velocity):
        u = np.asarray(commands, float)
        actual = np.asarray(measured_lengths, float)
        rate = np.asarray(measured_velocity, float)
        if (
            u.shape != (len(self.x), 4)
            or actual.shape != self.x.shape
            or rate.shape != self.x.shape
            or not np.isfinite(u).all()
            or np.any(abs(u) > 1)
            or not np.isfinite(actual).all()
            or not np.isfinite(rate).all()
            or not np.isfinite(dt)
            or dt <= 0
        ):
            raise ValueError("Invalid normalized actuator command/state")
        target = u[:, 1:] * self.vmax
        decay = np.exp(-dt / self.tau)
        # Exact solution for constant command over a step, including x integral.
        self.x += target * dt + (self.v - target) * self.tau * (1 - decay)
        self.v = target + (self.v - target) * decay
        self.x = np.clip(self.x, self.lo, self.hi)
        self.v = np.where(
            ((self.x <= self.lo) & (self.v < 0)) | ((self.x >= self.hi) & (self.v > 0)),
            0,
            self.v,
        )
        error = self.x - actual
        proposed = np.clip(
            self.integral_force + self.ki * error * dt, -self.fmax, self.fmax
        )
        trial = self.kp * error + self.kd * (self.v - rate) + proposed
        # Anti-windup: do not accumulate into saturation in the same direction.
        self.integral_force = np.where(
            (abs(trial) > self.fmax) & (trial * error > 0),
            self.integral_force,
            proposed,
        )
        force = np.clip(
            self.kp * error + self.kd * (self.v - rate) + self.integral_force,
            -self.fmax,
            self.fmax,
        )
        d = self.p["actuators"]["drive"]
        self.drive += (u[:, 0] * d["vmax"] - self.drive) * (1 - np.exp(-dt / d["tau"]))
        return force.copy(), self.drive.copy()


def normalized_reference(p, reference, angles):
    """Existing waypoint controller -> normalized velocity commands, not joint writes."""
    actual = cylinder_geometry(p, angles)[0]
    desired = cylinder_geometry(
        p, [reference.articulation_angle, reference.boom_angle, reference.tilt_angle]
    )[0]
    vmax = np.array([p["actuators"][n]["vmax"] for n in NAMES])
    feedback = p["actuators"]["reference_gain"]
    return np.r_[
        np.clip(reference.rear_axle_speed / p["actuators"]["drive"]["vmax"], -1, 1),
        np.clip(feedback * (desired - actual) / vmax, -1, 1),
    ]


def joint_efforts(p, actuator, normalized, q, qd, names, dt):
    """Map cylinder forces by virtual work; wheel torques use a bounded motor proxy."""
    from .commands import wheel_rates

    ids = [names.index(n) for n in ("articulation_joint", "boom_joint", "tilt_joint")]
    angles = np.asarray(q)[ids]
    vel = np.asarray(qd)[ids]
    lengths, jac, transmission = cylinder_geometry(p, angles)
    rate = jac @ vel
    if "cylinder_bodies" in p:
        measured_names = [
            "steering_left_slider_joint",
            "lift_left_slider_joint",
            "bucket_slider_joint",
        ]
        lengths = np.array(
            [
                p["cylinder_bodies"][n]["reference_length"]
                for n in ("steering_left", "lift_left", "bucket")
            ]
        ) + np.array([q[names.index(n)] for n in measured_names])
        rate = np.array([qd[names.index(n)] for n in measured_names])
    forces, drive = actuator.step(
        np.asarray(normalized)[None, :], dt, lengths[None, :], rate[None, :]
    )
    effort = np.zeros(len(names))
    if "cylinder_bodies" in p:
        # Physical linear forces, not equivalent joint torques. Passive cylinder
        # hinges and tip constraints transmit the reaction through the mechanism.
        for name, spec in p["cylinder_bodies"].items():
            channel = NAMES.index(spec["kind"])
            sign = -1 if name == "steering_right" else 1
            effort[names.index(name + "_slider_joint")] = sign * forces[0, channel]
    else:
        torque = transmission.T @ forces[0]
        for i, section in zip(ids, ("articulation", "boom", "linkage")):
            effort[i] = np.clip(
                torque[ids.index(i)],
                -p[section]["max_torque"],
                p[section]["max_torque"],
            )
    # Normalized drive reaches a first-order motor speed reference, never a state write.
    art = float(
        np.clip(
            angles[0], p["articulation"]["min_angle"], p["articulation"]["max_angle"]
        )
    )
    rates = wheel_rates(p, float(drive[0]), art, float(vel[0]))
    for name, value in rates.items():
        i = names.index(name)
        effort[i] = np.clip(
            p["solver"]["wheel_damping"] * (value - qd[i]),
            -p["wheels"]["max_torque"],
            p["wheels"]["max_torque"],
        )
    return effort, {
        "normalized": np.asarray(normalized).copy(),
        "force_N": forces[0].copy(),
        "reference_length_m": actuator.x[0].copy(),
        "actual_length_m": lengths.copy(),
    }
