"""Exclusive material-ID accounting, separate from physical payload dynamics."""

import numpy as np


def inside_bucket(points_local, profile, half_width, radii):
    """Center-in-cavity test with an explicit measurement clearance.

    The open lip-to-back-top segment is the measurement mouth plane. It is not
    a physical lid. Each included center contributes its whole element mass; surface protrusion
    is possible and no fractional boundary volume is inferred.
    """
    p = np.asarray(points_local)
    polygon = np.asarray(profile)
    x, z = p[:, 0], p[:, 2]
    inside = np.zeros(len(p), dtype=bool)
    distance = np.full(len(p), np.inf)
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        crossing = (a[1] > z) != (b[1] > z)
        if b[1] != a[1]:
            inside ^= crossing & (x < (b[0] - a[0]) * (z - a[1]) / (b[1] - a[1]) + a[0])
        d = b - a
        projection = np.clip(((p[:, [0, 2]] - a) @ d) / (d @ d), 0, 1)
        distance = np.minimum(
            distance, np.linalg.norm(p[:, [0, 2]] - a - projection[:, None] * d, axis=1)
        )
    return inside & (abs(p[:, 1]) + radii < half_width) & (distance > radii)


def bucket_payload_mask(points_local, parameters):
    """One centroid/plate-clearance definition shared by accounting and replay."""
    thickness = parameters["bucket"]["plate_thickness"]
    return inside_bucket(
        points_local,
        parameters["bucket"]["profile"],
        parameters["vehicle"]["bucket_width"] / 2 - thickness / 2,
        np.full(len(points_local), thickness / 2),
    )


class MaterialAccounting:
    def __init__(self, masses):
        self.masses = np.asarray(masses, dtype=float).copy()
        if (
            self.masses.ndim != 1
            or not np.isfinite(self.masses).all()
            or np.any(self.masses <= 0)
        ):
            raise ValueError("Every material ID needs a finite positive mass")
        self.initial = float(self.masses.sum())
        self.ever_acquired = np.zeros(len(masses), dtype=bool)

    def sample(self, in_bucket, at_source, lost, delivered=None):
        masks = [np.asarray(v, dtype=bool) for v in (in_bucket, at_source, lost)]
        if any(v.shape != self.masses.shape for v in masks):
            raise ValueError("Missing or duplicate material IDs")
        bucket, source, loss = masks
        destination = (
            np.zeros_like(bucket)
            if delivered is None
            else np.asarray(delivered, dtype=bool)
        )
        if destination.shape != bucket.shape or np.any(destination & bucket):
            raise ValueError("Invalid delivery classification")
        self.ever_acquired |= bucket & ~loss
        delivered_mask = destination & ~loss
        bucket_mask = bucket & ~loss & ~delivered_mask
        remaining = ~bucket_mask & ~loss & ~delivered_mask & ~self.ever_acquired
        spilled = self.ever_acquired & ~loss & ~delivered_mask & ~bucket_mask
        weights = lambda mask: float(self.masses[mask].sum())
        result = {
            "initial_mass_kg": self.initial,
            "remaining_mass_kg": weights(remaining),
            "remaining_in_source_zone_mass_kg": weights(remaining & source),
            "displaced_unacquired_mass_kg": weights(remaining & ~source),
            "bucket_mass_kg": weights(bucket_mask),
            "spilled_mass_kg": weights(spilled),
            "delivered_mass_kg": weights(delivered_mask),
            "boundary_loss_mass_kg": weights(loss),
            "cumulative_acquired_mass_kg": weights(self.ever_acquired),
            "bucket_exit_mass_kg": weights(
                self.ever_acquired & ~bucket_mask & ~loss & ~delivered_mask
            ),
        }
        result["conservation_residual_kg"] = self.initial - sum(
            result[k]
            for k in (
                "remaining_mass_kg",
                "bucket_mass_kg",
                "spilled_mass_kg",
                "delivered_mass_kg",
                "boundary_loss_mass_kg",
            )
        )
        return result
