"""Separate bounded trajectory diagnostic; does not modify the M2 controller."""

import numpy as np


class BoundedTrajectoryFeedback:
    def __init__(self, q, velocity, feedforward, kp, kd, limits):
        self.q, self.velocity, self.feedforward = map(
            np.asarray, (q, velocity, feedforward)
        )
        self.kp, self.kd, self.limits = map(np.asarray, (kp, kd, limits))
        if (
            self.q.shape != self.velocity.shape
            or self.q.shape != self.feedforward.shape
        ):
            raise ValueError("Feedback reference dimensions differ")
        if not all(
            np.isfinite(a).all()
            for a in (
                self.q,
                self.velocity,
                self.feedforward,
                self.kp,
                self.kd,
                self.limits,
            )
        ) or np.any(self.limits <= 0):
            raise ValueError("Invalid feedback parameters")

    def effort(self, tick, q, velocity):
        return np.clip(
            self.feedforward[tick]
            + self.kp * (self.q[tick] - q)
            + self.kd * (self.velocity[tick] - velocity),
            -self.limits,
            self.limits,
        )
