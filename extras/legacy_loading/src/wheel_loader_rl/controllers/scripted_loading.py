"""Open-loop feasibility sequence; no autonomous loading or driving policy."""
import numpy as np

# Time in seconds; joint targets: travel metres, lift radians, curl radians.
# Piecewise linear segments respect the default 1 m/s, .4 rad/s, .6 rad/s limits.
KEYFRAMES = np.array([
    [0, 0, 0, 0], [8, 0, 0, 0], [12, 3.4, 0, 0],
    [14, 3.4, 0.15, 0.65], [16, 3.4, 0.55, 0.65],
    [20.5, -0.9, 0.55, 0.65], [22, -0.9, 0.55, 0.65],
    [26, -0.9, 0.55, -1.1], [30, -0.9, 0.55, -1.1],
], dtype=float)


def loading_targets(time_s):
    """Return [travel, lift, curl] targets, clamped to the probe timeline."""
    if not np.isfinite(time_s) or time_s < 0:
        raise ValueError('time_s must be finite and nonnegative')
    return np.array([np.interp(time_s, KEYFRAMES[:, 0], KEYFRAMES[:, i]) for i in (1, 2, 3)])


def loading_phase(time_s):
    for end, label in ((8, 'settle'), (12, 'approach'), (14, 'curl'), (16, 'lift'),
                       (20.5, 'withdraw'), (22, 'retain'), (26, 'dump')):
        if time_s < end:
            return label
    return 'release'


def validate_loading_limits(limits):
    """Reject incompatible configured command rates before starting physics."""
    rates = abs(np.diff(KEYFRAMES[:, 1:], axis=0) / np.diff(KEYFRAMES[:, 0])[:, None])
    if np.any(rates > np.asarray(limits.scales) + 1e-9):
        raise ValueError('scripted probe exceeds configured longitudinal/lift/curl rate limits')
