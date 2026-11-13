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
