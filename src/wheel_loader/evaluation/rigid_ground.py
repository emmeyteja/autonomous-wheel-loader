"""Offline checks for the 15-second rigid-ground diagnostic, in SI units."""

import numpy as np


def evaluate_ground(p, times, positions, roots_xyzw, gaps, names):
    """Read control-tick samples; root poses are rear-frame world XYZ/XYZW."""
    times, positions, roots, gaps = map(
        np.asarray, (times, positions, roots_xyzw, gaps)
    )
    count = len(times)
    if (
        times.ndim != 1
        or count < 2
        or positions.shape != (count, len(names))
        or roots.shape != (count, 7)
        or gaps.shape != (count,)
        or any(
            not np.isfinite(value).all() for value in (times, positions, roots, gaps)
        )
        or not np.all(np.diff(times) > 0)
        or not np.allclose(np.linalg.norm(roots[:, 3:], axis=1), 1, atol=1e-4)
    ):
        raise ValueError(
            "Invalid ground trajectory dimensions, timestamps or finite poses"
        )
    straight = (times >= 2) & (times < 8)
    turning = times >= 11
    if straight.sum() < 2 or turning.sum() < 2 or times[-1] < 14.9:
        raise ValueError("Incomplete straight/turning diagnostic trajectory")
    x, y, z, w = roots[:, 3:].T
    yaw = np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    limits = all(
        positions[:, names.index(joint)].min() >= p[section]["min_angle"] - 0.005
        and positions[:, names.index(joint)].max() <= p[section]["max_angle"] + 0.005
        for joint, section in [
            ("articulation_joint", "articulation"),
            ("boom_joint", "boom"),
            ("tilt_joint", "linkage"),
            ("bucket_joint", "bucket"),
        ]
    )
    wheels = [
        f"{axle}_{side}_wheel_joint"
        for axle in ("front", "rear")
        for side in ("left", "right")
    ]
    metrics = {
        "forward_displacement_m": float(roots[-1, 0] - roots[0, 0]),
        "straight_max_heading_rad": float(np.max(np.abs(yaw[straight]))),
        "straight_lateral_drift_m": float(np.ptp(roots[straight, 1])),
        "sustained_turn_yaw_change_rad": float(yaw[turning][-1] - yaw[turning][0]),
        "maximum_loop_gap_m": float(np.max(gaps)),
    }
    return {
        "checks": {
            "driving": metrics["forward_displacement_m"] > 0.5,
            "straight_heading": metrics["straight_max_heading_rad"] < 0.05,
            "straight_lateral_drift": metrics["straight_lateral_drift_m"] < 0.1,
            "ground_turning": metrics["sustained_turn_yaw_change_rad"] > 0.05,
            "loop_closure": metrics["maximum_loop_gap_m"] < 0.005,
            "joint_limits": bool(limits),
            "wheel_rotation": all(
                np.ptp(positions[:, names.index(name)]) > 0.5 for name in wheels
            ),
        },
        **metrics,
    }


def evaluate_wheel_support(times, forces):
    """Check four upward contact forces at control ticks after two-second settling."""
    times, forces = np.asarray(times), np.asarray(forces)
    if forces.shape != (len(times), 4, 3) or not np.isfinite(forces).all():
        raise ValueError("Expected finite world contact forces for exactly four wheels")
    settled = times >= 2
    if not np.any(settled):
        raise ValueError("No settled force samples")
    fraction = float(np.mean(np.all(forces[settled, :, 2] > 1.0, axis=1)))
    return {
        "four_wheel_force_support_fraction_after_settling": fraction,
        "mean_total_upward_contact_force_N": float(
            forces[settled, :, 2].sum(axis=1).mean()
        ),
        "sustained_wheel_support": fraction >= 0.9,
    }
