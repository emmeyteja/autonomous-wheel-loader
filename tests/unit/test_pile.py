import numpy as np
from wheel_loader_rl.config import PileConfig
from wheel_loader_rl.pile import generate_pile


def test_seeded_layout_reproducible_without_global_rng_mutation():
    cfg = PileConfig()
    np.random.seed(12)
    before = np.random.get_state()
    first = generate_pile(cfg, 42)
    after = np.random.get_state()
    np.testing.assert_array_equal(before[1], after[1])
    assert before[2:] == after[2:]
    np.testing.assert_array_equal(first, generate_pile(cfg, 42))
    assert not np.array_equal(first, generate_pile(cfg, 43))


def test_spheres_start_finite_nonoverlapping_and_above_ground():
    cfg = PileConfig()
    centers = generate_pile(cfg, 42)
    assert centers.ndim == 2 and centers.shape[1] == 3 and len(centers) > 0
    assert centers.dtype == np.float32 and np.isfinite(centers).all()
    assert np.all(centers[:, 2] >= cfg.radius_m)
    differences = centers[:, None] - centers[None, :]
    distances = np.linalg.norm(differences, axis=-1)
    np.fill_diagonal(distances, np.inf)
    assert distances.min() >= 2 * cfg.radius_m - 1e-6
    assert cfg.sphere_mass_kg > 0


def test_trapezoid_spheres_fit_slope_faces_and_flat_top():
    import math
    from wheel_loader_rl.config import load_config
    from pathlib import Path
    cfg = load_config(Path(__file__).parents[2] / 'configs/pile_trapezoid_dense.toml').pile
    points = generate_pile(cfg, 42)
    x = np.abs(points[:, 0] - cfg.center_x_m)
    y = np.abs(points[:, 1])
    tangent = math.tan(math.radians(cfg.slope_deg))
    assert np.all(points[:, 2] >= cfg.radius_m - 1e-6)
    assert np.all(points[:, 2] + cfg.radius_m <= cfg.height_m + 1e-6)
    # Whole spheres, not just their centers, fit below every inclined face.
    clearance = cfg.radius_m * math.sqrt(1 + tangent**2)
    assert np.all(points[:, 2] + clearance <= (cfg.half_length_m - x) * tangent + 1e-6)
    assert np.all(points[:, 2] + clearance <= (cfg.half_width_m - y) * tangent + 1e-6)
    differences = points[:, None] - points[None, :]
    distances = np.linalg.norm(differences, axis=-1)
    np.fill_diagonal(distances, np.inf)
    assert distances.min() >= 2 * cfg.radius_m - 1e-6
    np.testing.assert_array_equal(points, generate_pile(cfg, 42))


def test_denser_preset_increases_resolution_without_resizing_the_stockpile():
    from pathlib import Path
    from wheel_loader_rl.config import load_config
    from wheel_loader_rl.pile import pile_geometry
    root = Path(__file__).parents[2]
    coarse = load_config(root / 'configs/pile_trapezoid.toml').pile
    dense = load_config(root / 'configs/pile_trapezoid_dense.toml').pile
    assert pile_geometry(coarse) == pile_geometry(dense)
    assert len(generate_pile(dense, 42)) > 2 * len(generate_pile(coarse, 42))
    assert dense.radius_m < coarse.radius_m


def test_particle_budget_rejects_instead_of_silently_deleting_material():
    from dataclasses import replace
    import pytest
    with pytest.raises(ValueError, match='max_particles'):
        generate_pile(replace(PileConfig(), max_particles=10), 42)


def test_huge_site_rejected_before_allocating_the_full_particle_grid():
    from dataclasses import replace
    import pytest
    with pytest.raises(ValueError, match='candidate-cell budget'):
        generate_pile(replace(PileConfig(), half_length_m=100, half_width_m=100, height_m=100), 42)
