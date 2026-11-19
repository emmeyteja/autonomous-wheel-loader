"""Shape/settling measurements, distinct from real-material validation."""

import numpy as np


def radial_surface(xyz, radii):
    """Fit four sector surface profiles. Sparse or flat profiles stay unavailable."""
    xyz, radii = np.asarray(xyz), np.asarray(radii)
    center = np.average(xyz[:, :2], axis=0, weights=radii**3)
    planar = xyz[:, :2] - center
    distance = np.linalg.norm(planar, axis=1)
    heading = np.arctan2(planar[:, 1], planar[:, 0])
    tops = xyz[:, 2] + radii
    width = float(2 * np.median(radii))
    maximum = float(np.max(tops))
    profiles = {}
    for name, angle in [
        ("front", 0),
        ("left", np.pi / 2),
        ("back", np.pi),
        ("right", -np.pi / 2),
    ]:
        sector = (
            np.abs(np.arctan2(np.sin(heading - angle), np.cos(heading - angle)))
            <= np.pi / 4
        )
        bins = []
        for lo in np.arange(0, np.quantile(distance, 0.98), width):
            mask = sector & (distance >= lo) & (distance < lo + width)
            if mask.sum() >= 3:
                height = float(np.quantile(tops[mask], 0.9))
                if 0.2 * maximum <= height <= 0.85 * maximum and height > 2.5 * width:
                    bins.append((lo + width / 2, height))
        if len(bins) < 4:
            profiles[name] = None
            continue
        x, y = np.asarray(bins).T
        slope, intercept = np.polyfit(x, y, 1)
        total = float(np.sum((y - y.mean()) ** 2))
        if slope >= 0 or total < 1e-10:
            profiles[name] = None
            continue
        profiles[name] = {
            "angle_deg": float(np.degrees(np.arctan(-slope))),
            "r_squared": float(1 - np.sum((y - slope * x - intercept) ** 2) / total),
            "bins": len(bins),
        }
    return profiles


def evaluate_column(times, xyz, velocities, radii, min_separation, warnings):
    times, xyz, velocities, radii = map(np.asarray, (times, xyz, velocities, radii))
    if (
        len(times) < 3
        or xyz.shape != velocities.shape
        or xyz.shape != (len(times), len(radii), 3)
        or not np.all(np.diff(times) > 0)
        or not all(np.isfinite(v).all() for v in (times, xyz, velocities, radii))
    ):
        raise ValueError("Invalid column recording")
    trailing = times >= times[-1] - 2
    if times[trailing][-1] - times[trailing][0] < 1.9:
        raise ValueError("Need at least two trailing seconds")
    speeds = np.linalg.norm(velocities[trailing], axis=-1)
    radial = np.linalg.norm(xyz[:, :, :2], axis=-1)
    drift = float(np.ptp(np.quantile(radial[trailing], 0.9, axis=1)))
    p95 = float(np.quantile(speeds, 0.95))
    checks = {
        "finite_states": True,
        "no_solver_warnings": warnings == 0,
        "all_element_ids_present": xyz.shape[1] == len(radii),
        "bounded_runout": bool(radial.max() < 1.0),
        "limited_overlap": min_separation >= -0.15 * float(np.min(radii)),
        "trailing_speed": p95 < 0.005,
        "trailing_runout_drift": drift < 0.005,
    }
    return {
        "checks": checks,
        "height_m": float(np.max(xyz[-1, :, 2] + radii)),
        "runout_radius_p95_m": float(np.quantile(radial[-1], 0.95)),
        "trailing_speed_p95_m_s": p95,
        "trailing_radius_p90_drift_m": drift,
        "minimum_contact_separation_m": float(min_separation),
        "surface_sector_fits": radial_surface(xyz[-1], radii),
        "reference_validation_passed": False,
        "reference_status": "No matched physical column-release data; not a field-test reproduction",
    }
