"""Deterministic procedural coarse-sphere pile generation (no physics)."""
import numpy as np
from wheel_loader_rl.config import PileConfig


def generate_pile(cfg: PileConfig, seed: int) -> np.ndarray:
    """Return sphere centers (particles, 3), float32, meters in scene frame.

    +X is the loading direction, +Y left, +Z up. Centers form a tapered
    elliptical mound. Spheres are above ground and initially non-overlapping.
    Uses a local RNG; it does not mutate training or global NumPy RNG state.
    Material density describes sphere solids, not calibrated bulk-soil density.
    """
    xs = np.arange(-cfg.half_length_m + cfg.radius_m, cfg.half_length_m, cfg.spacing_m)
    ys = np.arange(-cfg.half_width_m + cfg.radius_m, cfg.half_width_m, cfg.spacing_m)
    zs = np.arange(cfg.radius_m + cfg.jitter_m, cfg.height_m, cfg.spacing_m)
    grid = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), axis=-1).reshape(-1, 3)
    surface = cfg.height_m * (1 - (grid[:, 0] / cfg.half_length_m)**2 - (grid[:, 1] / cfg.half_width_m)**2)
    centers = grid[grid[:, 2] + cfg.radius_m + cfg.jitter_m <= surface].copy()
    if not len(centers):
        raise ValueError('pile configuration generates no material particles')
    centers += np.random.default_rng(seed).uniform(-cfg.jitter_m, cfg.jitter_m, centers.shape)
    centers[:, 0] += cfg.center_x_m
    return centers.astype(np.float32)
