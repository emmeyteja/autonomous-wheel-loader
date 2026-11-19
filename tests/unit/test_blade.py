"""Independent controls for bed preparation, SI force transforms and fit semantics."""

import numpy as np
import pytest

from wheel_loader.evaluation.blade import contact_wrench_on_geom, fit_draft_force
from wheel_loader.terrain.bed import RectangularBed


def test_force_intercept_is_not_mean_force_and_preserves_reaction_sign():
    x = np.linspace(0, 0.08, 81)
    reaction = -(12 + 80 * x)
    result = fit_draft_force(x, reaction, [0.02, 0.07])
    assert result["H0_N"] == pytest.approx(12)
    assert result["slope_N_per_m"] == pytest.approx(80)
    assert result["residual_rms_N"] < 1e-12
    assert not result["physical_validation_passed"]
    with pytest.raises(ValueError):
        fit_draft_force(x[::-1], reaction[::-1], [0.02, 0.07])


def test_contact_wrench_rotates_and_shifts_moment_and_reverses_sides():
    # Local normal maps to world +Y. Moment includes the contact lever arm.
    frame = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    force, moment = contact_wrench_on_geom(
        3, 7, frame, [10, 0, 0, 0, 0, 2], [1, 0, 0], np.zeros(3), 7
    )
    np.testing.assert_allclose(force, [0, 10, 0])
    np.testing.assert_allclose(moment, [0, 0, 12])
    f2, m2 = contact_wrench_on_geom(
        3, 7, frame, [10, 0, 0, 0, 0, 2], [1, 0, 0], np.zeros(3), 3
    )
    np.testing.assert_allclose(f2, -force)
    np.testing.assert_allclose(m2, -moment)


@pytest.mark.parametrize("radius", [0.0125, 0.01])
def test_rectangular_bed_keeps_mass_bounds_and_nonoverlap(radius):
    bed = RectangularBed(radius_m=radius)
    points, radii = bed.generate()
    assert np.sum(4 / 3 * np.pi * radii**3) * 2590 == pytest.approx(10)
    lo = points - radii[:, None]
    hi = points + radii[:, None]
    np.testing.assert_allclose(lo.min(0), [0.04, -0.14, 0], atol=1e-12)
    np.testing.assert_allclose(hi.max(0), [0.29, 0.14, 0.12], atol=1e-12)
    for start in range(0, len(radii), 100):
        gap = (
            np.linalg.norm(points[start : start + 100, None] - points[None], axis=-1)
            - radii[start : start + 100, None]
            - radii[None]
        )
        gap[np.arange(len(gap)), start + np.arange(len(gap))] = np.inf
        assert gap.min() > 0
    again, sizes = bed.generate()
    np.testing.assert_array_equal(again, points)
    np.testing.assert_array_equal(sizes, radii)


def test_bed_rejects_budget_and_inflation():
    with pytest.raises(ValueError, match="budget"):
        RectangularBed(max_particles=1).generate()
    with pytest.raises(ValueError, match="inflation"):
        RectangularBed(solid_volume_m3=1).generate()
