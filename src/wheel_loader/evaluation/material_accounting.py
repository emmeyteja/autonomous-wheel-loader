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
        for name, mask in (
            ("initial", np.ones_like(bucket)),
            ("remaining", remaining),
            ("remaining_in_source_zone", remaining & source),
            ("displaced_unacquired", remaining & ~source),
            ("bucket", bucket_mask),
            ("spilled", spilled),
            ("delivered", delivered_mask),
            ("boundary_loss", loss),
            ("cumulative_acquired", self.ever_acquired),
        ):
            result[name + "_count"] = int(mask.sum())
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


class AcquisitionFunnel:
    """Observe fixed IDs crossing the open mouth; contact never assigns payload.

    Inputs are successive measured bucket-local centres at control ticks. A
    swept centre segment crossing the finite mouth is an entry/exit observation,
    not a swept-sphere collision model. Wall-only classifier changes are exposed
    separately and cannot satisfy the physical-spillage gate.
    """

    def __init__(self, masses, parameters):
        self.masses = np.asarray(masses, float)
        if (
            self.masses.ndim != 1
            or not len(self.masses)
            or not np.isfinite(self.masses).all()
            or np.any(self.masses <= 0)
        ):
            raise ValueError("Every funnel ID needs a finite positive mass")
        profile = np.asarray(parameters["bucket"]["profile"], dtype=float)
        self.a, self.b = profile[-1], profile[0]
        self.edge = self.b - self.a
        self.normal = np.array([-self.edge[1], self.edge[0]])
        self.normal *= np.sign((profile.mean(axis=0) - self.a) @ self.normal)
        self.normal /= np.linalg.norm(self.normal)
        self.half_width = parameters["vehicle"]["bucket_width"] / 2
        self.previous = None
        self.previous_inside = np.zeros(len(masses), bool)
        self.contacted = np.zeros(len(masses), bool)
        self.entered = np.zeros(len(masses), bool)
        self.acquired = np.zeros(len(masses), bool)
        self.exited = np.zeros(len(masses), bool)
        self.extraction_spilled = np.zeros(len(masses), bool)
        self.unverified_entry = np.zeros(len(masses), bool)
        self.breakout = None
        self.crossings_in = self.crossings_out = 0
        self.events = []

    def sample(self, local, inside, contacting, phase, time_s=None):
        local, inside = np.asarray(local), np.asarray(inside, bool)
        contacting = np.asarray(contacting, bool)
        if (
            local.shape != (len(self.masses), 3)
            or inside.shape != self.masses.shape
            or contacting.shape != self.masses.shape
            or not np.isfinite(local).all()
        ):
            raise ValueError("Funnel observation must preserve every material ID")
        self.contacted |= contacting
        crossed_in = np.zeros(len(inside), bool)
        crossed_out = np.zeros(len(inside), bool)
        if self.previous is not None:
            before = (self.previous[:, [0, 2]] - self.a) @ self.normal
            after = (local[:, [0, 2]] - self.a) @ self.normal
            denominator = before - after
            fraction = np.divide(
                before,
                denominator,
                out=np.zeros_like(before),
                where=abs(denominator) > 1e-14,
            )
            cross = self.previous + fraction[:, None] * (local - self.previous)
            along = ((cross[:, [0, 2]] - self.a) @ self.edge) / (self.edge @ self.edge)
            aperture = (
                (fraction >= 0)
                & (fraction <= 1)
                & (along >= 0)
                & (along <= 1)
                & (abs(cross[:, 1]) < self.half_width)
            )
            crossed_in = aperture & (before <= 0) & (after > 0)
            crossed_out = aperture & (before > 0) & (after <= 0) & self.acquired
        self.entered |= crossed_in
        self.acquired |= inside & self.entered
        self.unverified_entry |= inside & ~self.entered
        self.exited |= crossed_out
        self.crossings_in += int(crossed_in.sum())
        self.crossings_out += int(crossed_out.sum())
        for event, mask in (
            ("mouth_entry", crossed_in),
            ("mouth_exit_after_acquisition", crossed_out),
        ):
            for i in np.flatnonzero(mask):
                self.events.append(
                    {
                        "event": event,
                        "id": int(i),
                        "mass_kg": float(self.masses[i]),
                        "time_s": time_s,
                        "phase": phase,
                        "centre_local_m": local[i].tolist(),
                    }
                )
        if phase in ("reverse", "spill_tip", "spill_hold", "stabilize"):
            if self.breakout is None:
                self.breakout = inside.copy()
            self.extraction_spilled |= crossed_out
        self.previous, self.previous_inside = local.copy(), inside.copy()

    def quantity(self, mask):
        return {"count": int(np.sum(mask)), "mass_kg": float(self.masses[mask].sum())}

    def report(self, inside, at_source, initial, final, radii, source_bounds):
        never = ~self.acquired
        sideways = abs(final[:, 1] - initial[:, 1]) > 2 * radii
        outside_lateral = (final[:, 1] < source_bounds[0][1]) | (
            final[:, 1] > source_bounds[1][1]
        )
        return {
            "initial_material": self.quantity(np.ones(len(inside), bool)),
            "contacted_by_bucket": self.quantity(self.contacted),
            "entered_mouth": self.quantity(self.entered),
            "acquired_inside_after_mouth_entry": self.quantity(self.acquired),
            "retained_after_breakout": self.quantity(
                self.breakout
                if self.breakout is not None
                else np.zeros(len(inside), bool)
            ),
            "spilled_during_extraction_unique": self.quantity(self.extraction_spilled),
            "ever_observed_mouth_exit_after_acquisition": self.quantity(self.exited),
            "final_payload": self.quantity(inside),
            "acquired_currently_outside_classifier": self.quantity(
                self.acquired & ~inside
            ),
            "classifier_inside_without_observed_mouth_entry": self.quantity(
                self.unverified_entry
            ),
            "never_acquired_displaced_sideways_over_diameter": self.quantity(
                never & sideways
            ),
            "never_acquired_outside_source_lateral_bounds": self.quantity(
                never & outside_lateral
            ),
            "never_acquired_remaining_in_source_region": self.quantity(
                never & at_source
            ),
            "never_acquired_remaining_total": self.quantity(never & ~inside),
            "never_acquired_displaced_outside_source_region": self.quantity(
                never & ~at_source & ~inside
            ),
            "mouth_entry_event_count": self.crossings_in,
            "mouth_exit_event_count_after_acquisition": self.crossings_out,
            "mouth_crossing_events": self.events,
            "observation": "240 Hz centre-segment aperture crossings; unique-ID masses; breakout is first reverse observation; exits are not displaced-pile labels",
        }
