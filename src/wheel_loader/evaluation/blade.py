"""SI tool-force processing; no simulator or material-calibration assumptions."""

import numpy as np


def fit_draft_force(travel_m, reaction_x_N, window_m):
    """Fit required horizontal force H=-Fx to H0 + slope*x on an explicit window."""
    x, force = np.asarray(travel_m), np.asarray(reaction_x_N)
    lo, hi = window_m
    if (
        x.ndim != 1
        or force.shape != x.shape
        or not np.isfinite(x).all()
        or not np.isfinite(force).all()
        or not (0 <= lo < hi)
    ):
        raise ValueError("Invalid force trace or fitting window")
    selected = (x >= lo) & (x <= hi)
    if (
        selected.sum() < 5
        or np.ptp(x[selected]) < 0.5 * (hi - lo)
        or np.any(np.diff(x[selected]) <= 0)
    ):
        raise ValueError("Insufficient forward travel for the force fit")
    slope, intercept = np.polyfit(x[selected], -force[selected], 1)
    residual = -force[selected] - (intercept + slope * x[selected])
    return {
        "H0_N": float(intercept),
        "slope_N_per_m": float(slope),
        "residual_rms_N": float(np.sqrt(np.mean(residual**2))),
        "fit_window_m": [float(lo), float(hi)],
        "sample_count": int(selected.sum()),
        "physical_validation_passed": False,
    }


def contact_wrench_on_geom(
    contact_geom1, contact_geom2, frame, local_wrench, position, origin, target
):
    """MuJoCo contact convention: positive local force acts on geom2.

    Return world force and moment about the target body's origin, both SI.
    This pure transform is separate from simulator contact collection.
    """
    if target not in (contact_geom1, contact_geom2):
        return np.zeros(3), np.zeros(3)
    sign = 1 if contact_geom2 == target else -1
    rotation = np.asarray(frame).reshape(3, 3).T
    force = sign * (rotation @ np.asarray(local_wrench)[:3])
    moment = sign * (rotation @ np.asarray(local_wrench)[3:]) + np.cross(
        np.asarray(position) - origin, force
    )
    return force, moment
