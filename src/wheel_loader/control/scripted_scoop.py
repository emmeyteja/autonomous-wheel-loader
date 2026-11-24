"""Deterministic single-scoop cylinder-velocity controller; no simulator imports."""

import numpy as np

from wheel_loader.control.cylinder_actuation import NAMES, cylinder_geometry
from wheel_loader.vehicle.kinematics import solve_linkage

PHASES = (
    "settle",
    "approach",
    "align_attack",
    "lower",
    "penetrate",
    "curl_advance",
    "lift_breakout",
    "reverse",
    "stabilize",
)


def attack_lengths(p, angle_deg, lip_height_m=0.055):
    """Plan physical cylinder lengths for edge pitch/height; no angle actuator."""
    if not np.isfinite(angle_deg) or not np.isfinite(lip_height_m) or lip_height_m <= 0:
        raise ValueError("Invalid attack workspace request")
    profile = np.asarray(p["bucket"]["profile"])
    edge = np.arctan2(*(profile[0] - profile[1])[::-1])
    target = np.array([np.deg2rad(angle_deg), lip_height_m])

    def residual(q):
        pose = solve_linkage(p, *q)
        return (
            np.array(
                [
                    pose.boom_elevation + pose.bucket_angle + edge,
                    pose.lip[1] + p["vehicle"]["articulation_height"],
                ]
            )
            - target
        )

    q = np.array(p["maneuver"]["lower_attack"], dtype=float)
    lower = np.array([p[n]["min_angle"] for n in ("boom", "linkage")])
    upper = np.array([p[n]["max_angle"] for n in ("boom", "linkage")])
    try:
        for _ in range(20):
            value = residual(q)
            if np.max(abs(value)) < 1e-8:
                break
            jac = np.stack(
                [(residual(q + np.eye(2)[j] * 1e-6) - value) / 1e-6 for j in range(2)],
                axis=1,
            )
            q = np.clip(q - np.linalg.solve(jac, value), lower, upper - 1e-6)
        if np.max(abs(residual(q))) > 1e-5:
            raise ValueError(
                "Requested attack pose is outside the planned linkage workspace"
            )
        lengths = cylinder_geometry(p, [0, *q])[0]
        if not all(
            p["cylinders"][n]["pin_distance_min"]
            <= x
            <= p["cylinders"][n]["pin_distance_max"]
            for n, x in zip(NAMES, lengths)
        ):
            raise ValueError("Requested attack pose exceeds cylinder workspace")
    except (ValueError, np.linalg.LinAlgError) as exc:
        raise ValueError(
            "Requested attack pose is outside the planned linkage workspace"
        ) from exc
    return lengths, q.tolist()


class ScriptedScoop:
    """Normalized [drive, steer, lift, curl], SI observation feedback.

    Cylinder targets are kinematic plans tracked by the existing bounded physical
    actuator. No body or joint state writes. One instance per experiment/reset.
    """

    def __init__(self, p, cfg, settle_s):
        self.p, self.cfg, self.settle_s = p, cfg, settle_s
        required = (
            "approach_speed_m_s",
            "approach_distance_m",
            "penetration_depth_m",
            "penetration_speed_m_s",
            "bucket_curl_velocity_m_s",
            "boom_lift_velocity_m_s",
            "breakout_threshold_N",
            "reverse_distance_m",
            "reverse_speed_m_s",
            "stabilize_s",
        )
        if not all(
            np.isfinite(cfg[k]) and cfg[k] > 0 for k in required
        ) or not np.isfinite(cfg["attack_angle_deg"]):
            raise ValueError("Invalid scoop controller parameters")
        self.attack, self.attack_pose = attack_lengths(
            p, cfg["attack_angle_deg"], cfg.get("attack_lip_height_m", 0.055)
        )
        self.transport = cylinder_geometry(p, [0, *p["maneuver"]["settle"]])[0]
        self.carry = cylinder_geometry(p, [0, *p["maneuver"]["curl_lift"]])[0]
        self.phases = PHASES
        if cfg.get("demonstrate_spillage", False):
            self.phases = PHASES[:-1] + ("spill_tip", "spill_hold", "stabilize")
        # Physical linkage targets, followed through bounded cylinder efforts.
        self.spill = (
            cylinder_geometry(p, [0, *p["maneuver"]["dump"]])[0]
            if cfg.get("demonstrate_spillage", False)
            else self.carry.copy()
        )
        self.index, self.entered, self.origin = 0, 0.0, 0.0
        if (
            not 0 < cfg.get("curl_start_fraction", 0.8) < 1
            or not np.isfinite(settle_s)
            or settle_s < 0
        ):
            raise ValueError("Invalid scoop phase parameters")
        self.last_time = -float("inf")
        self.last_x = None
        self.filtered_speed = 0.0
        self.drive_integral = 0.0
        self.done, self.failed, self.events = False, False, []
        self.entry_x = None
        self.advance_blocked = False
        self.load_response = cfg.get("load_response", "legacy")
        if self.load_response not in ("legacy", "stop_and_curl"):
            raise ValueError("Unknown scoop load response")

    @property
    def phase(self):
        return self.phases[self.index]

    def command(self, t, x, lengths, bucket_force_N, force_time_s=None):
        lengths = np.asarray(lengths, dtype=float)
        if (
            lengths.shape != (3,)
            or not np.isfinite(lengths).all()
            or not all(np.isfinite(v) for v in (t, x, bucket_force_N))
            or t < self.last_time
        ):
            raise ValueError("Invalid scoop observation or nonmonotonic time")
        if force_time_s is not None and (
            not np.isfinite(force_time_s) or force_time_s > t + 1e-8
        ):
            raise ValueError("Invalid reaction observation timestamp")
        dt = t - self.last_time if np.isfinite(self.last_time) else 0.0
        if self.last_x is not None and dt > 0:
            measured_speed = (x - self.last_x) / dt
            self.filtered_speed += (measured_speed - self.filtered_speed) * (
                1 - np.exp(-dt / 0.2)
            )
        self.last_time, self.last_x = t, x
        cfg = self.cfg
        age = t - self.entered
        target = (
            self.transport.copy()
            if self.index < 2
            else self.attack.copy()
            if self.index < 5
            else self.carry.copy()
        )
        speed = 0.0
        reached = False
        if self.phase == "settle":
            reached = age >= self.settle_s
        elif self.phase == "approach":
            speed = cfg["approach_speed_m_s"]
            reached = x - self.origin >= cfg["approach_distance_m"]
        elif self.phase == "align_attack":
            target[1] = self.transport[1]
            reached = abs(lengths[2] - target[2]) < 0.006 and age > 0.5
        elif self.phase == "lower":
            reached = np.max(abs(lengths[1:] - self.attack[1:])) < 0.008 and age > 0.5
        elif self.phase in ("penetrate", "curl_advance"):
            speed = cfg["penetration_speed_m_s"]
            if self.load_response == "stop_and_curl":
                blocked = (
                    bucket_force_N >= cfg["breakout_threshold_N"]
                    or x - self.entry_x >= cfg["penetration_depth_m"]
                )
                if blocked and not self.advance_blocked:
                    self.advance_blocked = True
                    self.drive_integral = 0.0
                if self.advance_blocked:
                    speed = 0.0  # Finite wheel braking; never a chassis state write.
            if self.phase == "penetrate":
                reached = (
                    x - self.origin
                    >= cfg["penetration_depth_m"] * cfg.get("curl_start_fraction", 0.8)
                    or bucket_force_N >= cfg["breakout_threshold_N"]
                )
            else:
                target[1] = self.attack[1]
                reached = (
                    abs(lengths[2] - self.carry[2]) < 0.006
                    if self.load_response == "stop_and_curl"
                    else x - self.entry_x >= cfg["penetration_depth_m"]
                    or bucket_force_N >= cfg["breakout_threshold_N"]
                )
        elif self.phase == "lift_breakout":
            reached = np.max(abs(lengths[1:] - self.carry[1:])) < 0.01 and age > 2
        elif self.phase == "reverse":
            speed = -cfg["reverse_speed_m_s"]
            reached = self.origin - x >= cfg["reverse_distance_m"]
        elif self.phase == "spill_tip":
            target = self.spill.copy()
            reached = np.max(abs(lengths[1:] - target[1:])) < 0.01 and age > 2
        elif self.phase == "spill_hold":
            target = self.spill.copy()
            reached = age >= cfg.get("spill_hold_s", 4.0)
        else:
            if cfg.get("demonstrate_spillage", False):
                target = self.spill.copy()
            reached = age >= cfg["stabilize_s"]
        vmax = np.array([self.p["actuators"][n]["vmax"] for n in NAMES])
        physical_velocity = np.clip(2 * (target - lengths), -vmax, vmax)
        physical_velocity[1] = np.clip(
            physical_velocity[1],
            -cfg["boom_lift_velocity_m_s"],
            cfg["boom_lift_velocity_m_s"],
        )
        physical_velocity[2] = np.clip(
            physical_velocity[2],
            -cfg["bucket_curl_velocity_m_s"],
            cfg["bucket_curl_velocity_m_s"],
        )
        if self.phase == "curl_advance":
            physical_velocity[2] = max(0.0, physical_velocity[2])
        # Outer chassis-speed feedback compensates the existing finite wheel
        # motor velocity servo under pile resistance. It changes commands only.
        error = speed - self.filtered_speed
        cap = self.p["actuators"]["drive"]["vmax"]
        candidate = (
            self.drive_integral + cfg.get("drive_integral_gain", 0.8) * error * dt
        )
        drive_reference = speed + cfg.get("drive_speed_gain", 2.0) * error + candidate
        if abs(drive_reference) < cap or drive_reference * error <= 0:
            self.drive_integral = candidate
        u = np.r_[np.clip(drive_reference / cap, -1, 1), physical_velocity / vmax]
        if age > 25:
            self.failed, self.done = True, True
        if reached and not self.done:
            self.events.append(
                {
                    "phase": self.phase,
                    "finished_s": float(t),
                    "x_m": float(x),
                    "bucket_reaction_N": float(bucket_force_N),
                    "reaction_sample_time_s": force_time_s,
                    "reaction_age_s": max(0.0, t - force_time_s)
                    if force_time_s is not None
                    else None,
                    "advance_blocked": self.advance_blocked,
                    "trigger": "measured_curl_target"
                    if self.phase == "curl_advance"
                    and self.load_response == "stop_and_curl"
                    else "reaction_force"
                    if self.phase in ("penetrate", "curl_advance")
                    and bucket_force_N >= cfg["breakout_threshold_N"]
                    else "target_reached",
                }
            )
            if self.phase == "lower":
                self.entry_x = x
            if self.index == len(self.phases) - 1:
                self.done = True
            else:
                self.index += 1
                self.entered, self.origin = t, x
        return np.clip(u, -1, 1)
