"""Gross double-acting cylinder force; only usable with known areas and pressures."""

import numpy as np


def gross_cylinder_force_N(
    cap_pressure_Pa, rod_pressure_Pa, bore_diameter_m, rod_diameter_m
):
    """Positive extension; excludes friction, backpressure not present in inputs, and linkage leverage."""
    values = np.array([bore_diameter_m, rod_diameter_m], float)
    if not np.isfinite(values).all() or not 0 < rod_diameter_m < bore_diameter_m:
        raise ValueError("Known positive bore/rod dimensions are required")
    cap, rod = np.asarray(cap_pressure_Pa, float), np.asarray(rod_pressure_Pa, float)
    if (
        cap.shape != rod.shape
        or not np.isfinite(cap).all()
        or not np.isfinite(rod).all()
        or np.any(cap < 0)
        or np.any(rod < 0)
    ):
        raise ValueError("Known nonnegative chamber pressures required")
    return (
        cap * np.pi * bore_diameter_m**2 / 4
        - rod * np.pi * (bore_diameter_m**2 - rod_diameter_m**2) / 4
    )
