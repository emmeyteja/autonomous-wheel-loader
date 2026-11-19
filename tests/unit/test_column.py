"""Independent geometry and metric controls for material-only diagnostics."""

import numpy as np
import pytest

from wheel_loader.evaluation.column import (
    evaluate_column,
    radial_surface,
    surface_profiles,
)
from wheel_loader.terrain.column import Column


def test_column_is_seeded_nonoverlapping_and_contained():
    column = Column(element_radius=0.02)
    xyz, radii = column.generate()
    repeated, sizes = column.generate()
    np.testing.assert_array_equal(xyz, repeated)
    np.testing.assert_array_equal(radii, sizes)
    gap = (
        np.linalg.norm(xyz[:, None] - xyz[None, :], axis=-1)
        - radii[:, None]
        - radii[None, :]
    )
    np.fill_diagonal(gap, np.inf)
    assert gap.min() > 0
    assert np.all(np.linalg.norm(xyz[:, :2], axis=1) + radii < column.radius)
    assert np.all(xyz[:, 2] - radii >= 0)
    assert np.all(xyz[:, 2] + radii <= column.height)


def test_column_budget_rejects_instead_of_discarding_grains():
    with pytest.raises(ValueError, match="budget"):
        Column(max_particles=5).generate()


def test_known_cone_recovers_slope_in_all_sectors():
    rng = np.random.default_rng(21)
    radius = np.sqrt(rng.uniform(0, 1, 20000))
    theta = rng.uniform(-np.pi, np.pi, len(radius))
    xyz = np.column_stack(
        (radius * np.cos(theta), radius * np.sin(theta), 1 - 0.7 * radius)
    )
    fits = radial_surface(xyz, np.full(len(xyz), 0.005))
    for fit in fits.values():
        assert fit is not None
        assert abs(fit["angle_deg"] - np.degrees(np.arctan(0.7))) < 1
        assert fit["r_squared"] > 0.99


def test_flat_stationary_layer_cannot_claim_material_validation():
    times = np.linspace(0, 4, 41)
    initial = np.column_stack(
        (np.linspace(-0.2, 0.2, 50), np.zeros(50), np.full(50, 0.01))
    )
    xyz = np.repeat(initial[None], len(times), axis=0)
    result = evaluate_column(times, xyz, np.zeros_like(xyz), np.full(50, 0.01), 0, 0)
    assert all(result["checks"].values())
    assert not result["reference_validation_passed"]
    assert all(fit is None for fit in result["surface_sector_fits"].values())
    # A moving layer fails the numerical screen even if positions are bounded.
    moving = evaluate_column(times, xyz, np.ones_like(xyz), np.full(50, 0.01), 0, 0)
    assert not moving["checks"]["trailing_speed"]


@pytest.mark.parametrize("radius,seed", [(0.0125, 42), (0.0125, 43), (0.01, 42)])
def test_controlled_column_matches_mass_and_envelope_without_overlap(radius, seed):
    column = Column(element_radius=radius, seed=seed, solid_volume_m3=13 / 2590)
    xyz, radii = column.generate()
    assert abs(np.sum(4 / 3 * np.pi * radii**3) * 2590 - 13) < 1e-10
    assert abs(np.max(xyz[:, 2] + radii) - column.height) < 1e-10
    assert abs(np.min(xyz[:, 2] - radii)) < 1e-10
    assert (
        abs(np.max(np.linalg.norm(xyz[:, :2], axis=1) + radii) - column.radius) < 1e-10
    )
    # Chunk the independent pair-distance check to bound test memory.
    for start in range(0, len(radii), 100):
        distance = np.linalg.norm(xyz[start : start + 100, None] - xyz[None], axis=-1)
        gap = distance - radii[start : start + 100, None] - radii[None]
        gap[np.arange(len(gap)), start + np.arange(len(gap))] = np.inf
        assert gap.min() > 0


def test_controlled_column_rejects_mass_that_requires_overlapping_inflation():
    with pytest.raises(ValueError, match="without inflating"):
        Column(solid_volume_m3=1).generate()


def test_scan_profiles_measure_heights_without_inventing_particle_radii():
    # Independently constructed 35-degree surface cloud in a declared level frame.
    # Duplicate apex/base rings are avoided so the profile samples remain regular.
    radial = np.linspace(0.02, 0.98, 100)
    heading = np.linspace(-np.pi, np.pi, 180, endpoint=False)
    r, a = np.meshgrid(radial, heading)
    cloud = np.column_stack(
        (
            r.ravel() * np.cos(a.ravel()),
            r.ravel() * np.sin(a.ravel()),
            (1 - r.ravel()) * np.tan(np.deg2rad(35)),
        )
    )
    profiles = surface_profiles(cloud, 0.04, center_xy=[0, 0])
    for fit in profiles.values():
        assert fit is not None
        assert abs(fit["angle_deg"] - 35) < 0.5
        assert fit["r_squared"] > 0.99
    # A datum below all elevations cannot silently be interpreted as a leveled scan.
    cloud[:, 2] -= 10
    assert all(fit is None for fit in surface_profiles(cloud, 0.04).values())
