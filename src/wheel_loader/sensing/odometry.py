"""Batched rear-axle odometry from wheel encoders and central articulation."""

import numpy as np


class ArticulatedOdometry:
    def __init__(self, count, wheel_radius_m, wheelbase_m):
        if count < 1 or wheel_radius_m <= 0 or wheelbase_m <= 0:
            raise ValueError("Invalid odometry dimensions")
        self.radius, self.halfbase = wheel_radius_m, wheelbase_m / 2
        self.pose = np.zeros(
            (count, 3)
        )  # Rear axle XY/yaw, each environment's local odom frame.
        self.previous = np.zeros(
            (count, 3)
        )  # rear left/right wheel angle, articulation
        self.initialized = np.zeros(count, dtype=bool)

    def reset(self, mask):
        mask = np.asarray(mask, bool)
        if mask.shape != self.initialized.shape:
            raise ValueError("Reset mask shape differs")
        self.pose[mask] = 0
        self.previous[mask] = 0
        self.initialized[mask] = False

    def update(self, wheel_angles_rad, articulation_rad):
        wheels, art = (
            np.asarray(wheel_angles_rad, float),
            np.asarray(articulation_rad, float),
        )
        b = len(self.pose)
        if (
            wheels.shape != (b, 2)
            or art.shape != (b,)
            or not np.isfinite(wheels).all()
            or not np.isfinite(art).all()
            or np.any(abs(art) >= np.pi / 2)
        ):
            raise ValueError("Invalid encoder/articulation batch")
        current = np.column_stack((wheels, art))
        delta = current - self.previous
        delta[~self.initialized] = 0
        if np.any(abs(delta[:, :2]) >= np.pi):
            raise ValueError("Encoder sampling too sparse or angles wrapped")
        ds = self.radius * delta[:, :2].mean(axis=1)
        amid = (art + self.previous[:, 2]) / 2
        dyaw = (ds * np.sin(amid) - self.halfbase * delta[:, 2]) / (
            self.halfbase * (1 + np.cos(amid))
        )
        # Exact constant-twist SE(2) arc; tends continuously to straight travel.
        travel = ds * np.sinc(dyaw / (2 * np.pi))
        heading = self.pose[:, 2] + dyaw / 2
        self.pose[:, 0] += travel * np.cos(heading)
        self.pose[:, 1] += travel * np.sin(heading)
        self.pose[:, 2] += dyaw
        self.previous = current.copy()
        self.initialized[:] = True
        return self.pose.copy()
