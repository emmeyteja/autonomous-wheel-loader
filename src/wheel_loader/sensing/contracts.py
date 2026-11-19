"""Owned sensor snapshots; SI units, batch-first dimensions and capture timestamps."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RangeScan:
    capture_time_s: np.ndarray  # [B], simulation clock
    arrival_time_s: np.ndarray  # [B], separate from measurement time
    ranges_m: np.ndarray  # [B,R], NaN where invalid
    valid: np.ndarray  # [B,R]
    directions_sensor: np.ndarray  # [R,3], unit vectors
    frame_id: str = "lidar"

    def __post_init__(self):
        for name in (
            "capture_time_s",
            "arrival_time_s",
            "ranges_m",
            "valid",
            "directions_sensor",
        ):
            value = np.array(getattr(self, name), copy=True)
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        b, r = self.ranges_m.shape
        if (
            self.valid.shape != (b, r)
            or self.valid.dtype != bool
            or self.capture_time_s.shape != (b,)
            or self.arrival_time_s.shape != (b,)
            or self.directions_sensor.shape != (r, 3)
            or not self.frame_id
        ):
            raise ValueError("Invalid range scan dimensions/frame")
        if (
            not np.isfinite(self.capture_time_s).all()
            or not np.isfinite(self.arrival_time_s).all()
            or np.any(self.arrival_time_s < self.capture_time_s)
            or not np.isfinite(self.directions_sensor).all()
            or not np.allclose(np.linalg.norm(self.directions_sensor, axis=1), 1)
            or not np.isfinite(self.ranges_m[self.valid]).all()
            or np.any(self.ranges_m[self.valid] <= 0)
            or not np.isnan(self.ranges_m[~self.valid]).all()
        ):
            raise ValueError("Invalid scan times/ranges/directions")

    def points_sensor_m(self):
        return self.ranges_m[..., None] * self.directions_sensor[None, :, :]


@dataclass(frozen=True)
class Availability:
    available: bool
    quantity: str
    unit: str
    provenance: str
    reason: str = ""

    def __post_init__(self):
        if not self.available and not self.reason:
            raise ValueError("An unavailable channel needs a reason")
