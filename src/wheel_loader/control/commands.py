"""SI commands for articulated chassis; steering angle is NOT an Ackermann angle."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LoaderCommand:
    rear_axle_speed: float = 0.0  # m/s, positive forward
    articulation_angle: float = 0.0  # rad, front yaw relative to rear; positive left
    boom_angle: float = 0.0  # rad from configured nominal elevation
    tilt_angle: float = (
        0.0  # rad bellcrank displacement; positive dumps on selected branch
    )

    def validate(self, p):
        values = (
            self.rear_axle_speed,
            self.articulation_angle,
            self.boom_angle,
            self.tilt_angle,
        )
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Commands must be finite")
        if (
            abs(self.rear_axle_speed)
            > p["wheels"]["max_velocity"] * p["vehicle"]["wheel_radius"]
        ):
            raise ValueError("Drive speed exceeds diagnostic bound")
        for section, value in [
            ("articulation", self.articulation_angle),
            ("boom", self.boom_angle),
            ("linkage", self.tilt_angle),
        ]:
            if not p[section]["min_angle"] <= value <= p[section]["max_angle"]:
                raise ValueError(f"{section} command exceeds configured bound")


def wheel_rates(p, rear_speed, angle, angle_rate=0.0):
    """No-slip planar reference for central articulation and equal axle offsets.

    rear yaw rate = (v_r sin(a) - L_front a_dot)/(L_rear cos(a)+L_front).
    Front/rear axle speeds account for changing articulation. Tire slip, power
    sharing and axle oscillation are not modeled by this reference controller.
    +Y wheel axes mean positive wheel rotation drives +X.
    """
    if not all(math.isfinite(v) for v in (rear_speed, angle, angle_rate)):
        raise ValueError("Wheel reference inputs must be finite")
    if not p["articulation"]["min_angle"] <= angle <= p["articulation"]["max_angle"]:
        raise ValueError("Articulation exceeds configured envelope")
    length = p["vehicle"]["wheelbase"] / 2
    yaw_rear = (rear_speed * math.sin(angle) - length * angle_rate) / (
        length * (1 + math.cos(angle))
    )
    yaw_front = yaw_rear + angle_rate
    front_speed = rear_speed * math.cos(angle) + length * yaw_rear * math.sin(angle)
    half_track, radius = p["vehicle"]["track_width"] / 2, p["vehicle"]["wheel_radius"]
    return {
        f"{axle}_{side}_wheel_joint": (speed - sign * half_track * yaw) / radius
        for axle, speed, yaw in [
            ("rear", rear_speed, yaw_rear),
            ("front", front_speed, yaw_front),
        ]
        for side, sign in [("left", 1), ("right", -1)]
    }
