"""Pure diagnostics for recorded particle motion; no calibrated-soil claims."""
import math
import numpy as np


def settling_metrics(positions, times, radius_m, window_s=2.0):
    """Use a trailing window, not a single final frame, to detect continuing flow.

    Positions: [frames, particles, xyz] in world metres; times: simulation seconds.
    Thresholds are engineering screening criteria, not a sand validation standard.
    """
    p, t = np.asarray(positions), np.asarray(times)
    if p.ndim != 3 or p.shape[-1] != 3 or p.shape[1] < 1 or len(p) < 2:
        raise ValueError('positions must have shape [frames>=2, particles>=1, 3]')
    if t.shape != (len(p),) or not np.isfinite(p).all() or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError('finite positions and strictly increasing matching times required')
    if not math.isfinite(radius_m) or radius_m <= 0 or not math.isfinite(window_s) or window_s <= 0:
        raise ValueError('radius_m and window_s must be finite and positive')
    start = max(0, int(np.searchsorted(t, t[-1] - window_s)) - 1)
    speeds = np.linalg.norm(np.diff(p[start:], axis=0), axis=-1) / np.diff(t[start:])[:, None]
    center = p[0, :, :2].mean(axis=0)
    radial = np.linalg.norm(p[[start, -1], :, :2] - center, axis=-1)
    drift = abs(float(np.diff(np.quantile(radial, 0.9, axis=1))[0]))
    p95 = float(np.quantile(speeds, 0.95))
    moving = float(np.mean(speeds > 0.05))
    duration = float(t[-1] - t[start])
    return {'window_s': duration, 'speed_p95_mps': p95, 'speed_max_mps': float(speeds.max()),
            'fraction_samples_above_0_05_mps': moving, 'radius_p90_drift_m': drift,
            'settling_screen_passed': bool(duration >= window_s and p95 <= 0.02 and moving <= 0.01 and drift <= 0.02),
            'screen_limits': {'speed_p95_mps': 0.02, 'fraction_above_0_05_mps': 0.01, 'radius_p90_drift_m': 0.02},
            'surface_slopes': surface_slopes(p[-1], radius_m)}


def surface_slopes(positions, radius_m):
    """Fit four central side profiles to sphere-top envelopes in diameter-wide bins.

    Excludes the flat top and ground layer (20–80% of peak height). Reports fit
    quality; sparse/short faces are unavailable. This is a resolution-dependent
    stockpile diagnostic, not a laboratory funnel angle-of-repose measurement.
    """
    p = np.asarray(positions)
    results = {}
    center = (np.quantile(p[:, :2], 0.1, axis=0) + np.quantile(p[:, :2], 0.9, axis=0)) / 2
    for axis, label in ((0, 'x'), (1, 'y')):
        cross = 1 - axis
        strip = abs(p[:, cross] - center[cross]) <= max(3 * radius_m, np.ptp(p[:, cross]) * 0.15)
        for sign in (-1, 1):
            d = sign * (p[:, axis] - center[axis])
            valid = strip & (d >= 0)
            points = []
            if valid.any():
                edges = np.arange(0, d[valid].max() + 2 * radius_m, 2 * radius_m)
                for lo, hi in zip(edges[:-1], edges[1:]):
                    inside = valid & (d >= lo) & (d < hi)
                    if inside.any():
                        points.append(((lo + hi) / 2, float(p[inside, 2].max() + radius_m)))
            profile = np.asarray(points).reshape(-1, 2)
            key = f'{label}{sign:+d}'
            results[key] = None
            if len(profile) < 3:
                continue
            peak = profile[:, 1].max()
            side = profile[(profile[:, 1] >= max(2.5 * radius_m, 0.2 * peak)) & (profile[:, 1] <= 0.8 * peak)]
            if len(side) < 3:
                continue
            slope, intercept = np.polyfit(side[:, 0], side[:, 1], 1)
            residual = np.sum((side[:, 1] - (slope * side[:, 0] + intercept))**2)
            total = np.sum((side[:, 1] - side[:, 1].mean())**2)
            results[key] = {'angle_deg': float(np.degrees(np.arctan(-slope))), 'bins': len(side),
                            'r_squared': float(1 - residual / total) if total > 0 else 0.0}
    return results
