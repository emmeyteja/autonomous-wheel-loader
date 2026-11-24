"""Pure measurements for aligned, immutable physics-backend replays (SI units)."""

import numpy as np


def envelope_penetration(lip, centres, radii, bucket_width):
    """X penetration into a common initial sphere envelope at lip height.

    Assumes a straight +X approach, level cutting edge along world Y. A sphere
    intersects the edge-height/lateral strip when dz² + dy_outside² <= radius².
    NaN means the edge misses that envelope, not zero penetration. This measures
    the *initial* envelope, not a current deformed continuum surface.
    """
    lip, centres, radii = map(np.asarray, (lip, centres, radii))
    output = np.full(len(lip), np.nan)
    for i, p in enumerate(lip):
        outside = np.maximum(abs(centres[:, 1] - p[1]) - bucket_width / 2, 0)
        cross = radii**2 - (centres[:, 2] - p[2]) ** 2 - outside**2
        hit = cross >= 0
        if hit.any():
            front = np.min(centres[hit, 0] - np.sqrt(cross[hit]))
            output[i] = max(0.0, float(p[0] - front))
    return output


def aligned(reference, candidate):
    """Reject partial/time-shifted comparisons rather than interpolating them."""
    a, b = reference["time_s"], candidate["time_s"]
    if a.shape != b.shape or not np.array_equal(a, b):
        raise ValueError("Benchmark sampling times differ")
    for states in (reference, candidate):
        for key in ("body_positions", "joint_pos"):
            if not np.isfinite(states[key]).all():
                raise ValueError("Nonfinite benchmark states")


def trajectory_error(reference, candidate):
    delta = np.asarray(candidate) - np.asarray(reference)
    distance = np.linalg.norm(delta, axis=-1)
    return {
        "rms_m": float(np.sqrt(np.mean(distance**2))),
        "max_m": float(distance.max()),
    }


def compare(reference, candidate, repeat):
    aligned(reference, candidate)
    aligned(candidate, repeat)
    return {
        "bucket_error": trajectory_error(
            reference["bucket_trajectory"], candidate["bucket_trajectory"]
        ),
        "boom_error": trajectory_error(
            reference["boom_trajectory"], candidate["boom_trajectory"]
        ),
        "articulation_rms_rad": float(
            np.sqrt(
                np.mean(
                    (reference["articulation_rad"] - candidate["articulation_rad"]) ** 2
                )
            )
        ),
        "repeat_max_body_position_difference_m": float(
            np.linalg.norm(
                candidate["body_positions"] - repeat["body_positions"], axis=-1
            ).max()
        ),
        "final_pile_error": trajectory_error(
            reference["final_pile"], candidate["final_pile"]
        ),
    }
