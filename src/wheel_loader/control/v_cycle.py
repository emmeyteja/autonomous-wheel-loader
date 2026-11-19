"""Feedback-controlled rigid-ground V-cycle motion rehearsal, without material."""

import math
from dataclasses import dataclass

import numpy as np

from .commands import LoaderCommand

PHASES = (
    "settle",
    "forward_right",
    "lower_attack",
    "push",
    "curl_lift",
    "reverse_loaded",
    "forward_left",
    "raise_dump",
    "dump",
    "rack_back",
    "reverse_home",
    "lower_finish",
    "stop",
)


@dataclass(frozen=True)
class VCycleScenario:
    name: str = "nominal"
    distance: float = 12.0
    half_width: float = 6.0
    speed: float = 0.6
    max_duration: float = 360.0

    def __post_init__(self):
        if any(
            not math.isfinite(v) or v <= 0
            for v in (self.distance, self.half_width, self.speed, self.max_duration)
        ):
            raise ValueError("V-cycle dimensions, speed and timeout must be positive")

    @classmethod
    def named(cls, name):
        return {
            "nominal": cls(),
            "slow": cls("slow", speed=0.45),
            "wide": cls("wide", distance=15.0, half_width=7.5),
        }[name]


class VCycleController:
    """Single-evaluation controller; observes rear-frame world XY/yaw in SI units.

    Only driven steering, boom, bellcrank and wheel references are commanded.
    The passive bucket and rod remain controlled by the physical linkage.
    """

    def __init__(self, parameters, scenario):
        self.p, self.scenario = parameters, scenario
        self.phase = 0
        self.entered = 0.0
        self.goal = None
        self.path_start = None
        self.done = False
        self.speed = 0.0
        self.steer = 0.0
        self.boom = 0.15
        self.tilt = parameters.get("maneuver", {}).get("settle", [0.15, -0.3])[1]
        self.events = []
        self.timeout = False
        self.raise_aligned = False
        self.recurl_aligned = False

    @property
    def phase_name(self):
        return PHASES[self.phase]

    def _advance(self, t, xy, yaw):
        self.events.append(
            {
                "phase": self.phase_name,
                "finished_s": float(t),
                "xy_m": [float(value) for value in xy],
            }
        )
        self.phase += 1
        self.entered = t
        self.goal = None
        self.path_start = None
        if self.phase == len(PHASES):
            self.phase -= 1
            self.done = True

    def _choose_steering(self, xy, yaw, articulation, gear, goal):
        """Sample articulated kinematics including steering-rate reaction at the hinge."""
        candidates = np.linspace(-0.6, 0.6, 25)
        x, y, heading, a = [
            np.full(len(candidates), v, dtype=float) for v in (*xy, yaw, articulation)
        ]
        length = self.p["vehicle"]["wheelbase"] / 2
        path = np.asarray(goal) - self.path_start
        norm = max(float(np.linalg.norm(path)), 1e-6)
        cost = np.zeros(len(candidates))
        for _ in range(50):
            distance = np.hypot(goal[0] - x, goal[1] - y)
            v = gear * np.minimum(self.scenario.speed, 0.5 * distance)
            v = np.where(distance < 0.12, 0.0, v)
            da = np.clip(candidates - a, -0.03, 0.03) / 0.2
            omega = (v * np.sin(a) - length * da) / (length * (1 + np.cos(a)))
            x += 0.2 * (v * np.cos(heading) - length * omega * np.sin(heading))
            y += 0.2 * (v * np.sin(heading) + length * omega * np.cos(heading))
            heading += 0.2 * omega
            a += 0.2 * da
            cross = (
                (x - self.path_start[0]) * path[1] - (y - self.path_start[1]) * path[0]
            ) / norm
            cost += 0.003 * cross**2
        cost += (goal[0] - x) ** 2 + (goal[1] - y) ** 2 + 0.01 * candidates**2
        return float(candidates[np.argmin(cost)])

    def command(self, t, dt, xy, yaw, articulation=None, attachment=None):
        if (
            not all(math.isfinite(v) for v in (t, dt, *xy, yaw))
            or dt <= 0
            or (articulation is not None and not math.isfinite(articulation))
        ):
            raise ValueError("Invalid V-cycle observation or timestep")
        if t >= self.scenario.max_duration and not self.done:
            self.timeout = True
            self.done = True
        phase = self.phase_name
        age = t - self.entered
        travel = {
            "forward_right": (1, (self.scenario.distance, -self.scenario.half_width)),
            "reverse_loaded": (-1, (0.0, 0.0)),
            "forward_left": (1, (self.scenario.distance, self.scenario.half_width)),
            "reverse_home": (-1, (0.0, 0.0)),
        }
        speed, steer = 0.0, 0.0
        boom, tilt = {
            "settle": (0.15, -0.3),
            "forward_right": (0.15, -0.3),
            "lower_attack": (0.02, 0.0),
            "push": (0.02, 0.0),
            "curl_lift": (0.45, -0.55),
            "reverse_loaded": (0.45, -0.55),
            "forward_left": (0.45, -0.55),
            "raise_dump": (0.65, -0.55),
            "dump": (0.45, 0.78),
            "rack_back": (0.45, -0.3),
            "reverse_home": (0.45, -0.3),
            "lower_finish": (0.15, -0.3),
            "stop": (0.15, -0.3),
        }[phase]
        if "maneuver" in self.p:
            boom, tilt = self.p["maneuver"][phase]
        if "actuators" in self.p and phase == "raise_dump" and attachment is not None:
            self.raise_aligned = self.raise_aligned or abs(attachment[1] - tilt) < 0.002
            if not self.raise_aligned:
                boom = self.p["maneuver"]["forward_left"][0]
        if "actuators" in self.p and phase == "rack_back":
            transport_tilt = self.p["maneuver"]["raise_dump"][1]
            self.recurl_aligned = self.recurl_aligned or (
                abs(
                    (self.tilt if attachment is None else attachment[1])
                    - transport_tilt
                )
                < 0.002
            )
            if not self.recurl_aligned:
                # Close the bucket to raised rollback before lowering the arm.
                boom = self.p["maneuver"]["raise_dump"][0]
                tilt = transport_tilt
        if phase == "push":
            speed = 0.15
        reached = False
        if phase in travel and not self.done:
            gear, goal = travel[phase]
            distance = math.hypot(goal[0] - xy[0], goal[1] - xy[1])
            if self.path_start is None:
                self.path_start = tuple(xy)
            if self.goal is None or t - self.goal[0] >= 0.1:
                steer = self._choose_steering(
                    xy,
                    yaw,
                    self.steer if articulation is None else articulation,
                    gear,
                    goal,
                )
                self.goal = (t, steer)
            steer = self.goal[1]
            speed = gear * min(self.scenario.speed, 0.5 * distance)
            reached = distance < 0.20
        durations = {
            "push": 4.0,
            "settle": 2.0,
            "lower_attack": 4.0,
            "curl_lift": 6.0,
            "raise_dump": 4.0,
            "dump": 7.0,
            "rack_back": 6.0,
            "lower_finish": 4.0,
            "stop": 3.0,
        }

        # Bounded command slew avoids impulsive steering and attachment targets.
        def slew(old, new, rate):
            return old + max(-rate * dt, min(rate * dt, new - old))

        self.speed = slew(self.speed, 0.0 if reached or self.done else speed, 0.2)
        self.steer = slew(self.steer, steer, 0.15)
        self.boom = slew(self.boom, boom, 0.10)
        self.tilt = slew(self.tilt, tilt, 0.04 if "actuators" in self.p else 0.22)
        result = LoaderCommand(self.speed, self.steer, self.boom, self.tilt)
        result.validate(self.p)
        attachment_ready = attachment is None or (
            abs(attachment[0] - boom) < 0.015 and abs(attachment[1] - tilt) < 0.002
        )
        if not self.done and (
            (reached and abs(self.speed) < 0.02)
            or (
                phase in durations
                and age >= durations[phase]
                and abs(self.boom - boom) < 1e-6
                and abs(self.tilt - tilt) < 1e-6
                and attachment_ready
            )
        ):
            self._advance(t, xy, yaw)
        return result
