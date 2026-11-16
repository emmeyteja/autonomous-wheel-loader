"""Deterministic procedural coarse-sphere pile generation (no physics)."""
import math
import numpy as np
from wheel_loader_rl.config import PileConfig

# Bound temporary CPU allocation before constructing a three-dimensional grid.
MAX_CANDIDATE_CELLS = 2_000_000


def generate_pile(cfg: PileConfig, seed: int) -> np.ndarray:
    """Return sphere centers (particles, 3), float32, meters in scene frame.

    +X is the loading direction, +Y left, +Z up. Mound and flat-topped
    trapezoidal stockpiles are supported. This defines initial geometry only;
    physics is free to rearrange it. Spheres start above ground and without overlaps.
    Uses a local RNG; it does not mutate training or global NumPy RNG state.
    Material density describes sphere solids, not calibrated bulk-soil density.
    """
    candidate_bound = (math.ceil(2 * cfg.half_length_m / cfg.spacing_m)
                       * math.ceil(2 * cfg.half_width_m / cfg.spacing_m)
                       * math.ceil(cfg.height_m / cfg.spacing_m))
    if candidate_bound > MAX_CANDIDATE_CELLS:
        raise ValueError('Pile generation exceeds the candidate-cell budget; use a smaller active region or coarser spacing')
    xs = np.arange(-cfg.half_length_m + cfg.radius_m, cfg.half_length_m, cfg.spacing_m)
    ys = np.arange(-cfg.half_width_m + cfg.radius_m, cfg.half_width_m, cfg.spacing_m)
    zs = np.arange(cfg.radius_m + cfg.jitter_m, cfg.height_m, cfg.spacing_m)
    grid = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), axis=-1).reshape(-1, 3)
    if cfg.shape == 'mound':
        surface = cfg.height_m * (1 - (grid[:, 0] / cfg.half_length_m)**2 - (grid[:, 1] / cfg.half_width_m)**2)
        inside = grid[:, 2] + cfg.radius_m + cfg.jitter_m <= surface
    else:
        tangent = math.tan(math.radians(cfg.slope_deg))
        # Erode each inclined face by a whole sphere radius plus the worst
        # possible x/y and z jitter, rather than checking centers alone.
        margin = cfg.radius_m * math.sqrt(1 + tangent**2) + cfg.jitter_m * (1 + tangent)
        inside = ((grid[:, 2] + cfg.radius_m + cfg.jitter_m <= cfg.height_m)
                  & (grid[:, 2] + margin <= (cfg.half_length_m - np.abs(grid[:, 0])) * tangent)
                  & (grid[:, 2] + margin <= (cfg.half_width_m - np.abs(grid[:, 1])) * tangent))
    centers = grid[inside].copy()
    if len(centers) > cfg.max_particles:
        raise ValueError(f'Pile needs {len(centers)} particles, above max_particles={cfg.max_particles}; use an active region or coarser spacing')
    if not len(centers):
        raise ValueError('pile configuration generates no material particles')
    centers += np.random.default_rng(seed).uniform(-cfg.jitter_m, cfg.jitter_m, centers.shape)
    centers[:, 0] += cfg.center_x_m
    return centers.astype(np.float32)


def pile_geometry(cfg: PileConfig) -> dict:
    """Describe intended initial geometry, not the settled pile's measured shape."""
    result = {'shape': cfg.shape, 'base_length_m': 2 * cfg.half_length_m,
              'base_width_m': 2 * cfg.half_width_m, 'height_m': cfg.height_m,
              'front_x_m': cfg.center_x_m - cfg.half_length_m}
    if cfg.shape == 'trapezoid':
        run = cfg.height_m / math.tan(math.radians(cfg.slope_deg))
        result.update(initial_slope_deg=cfg.slope_deg,
                      top_length_m=2 * (cfg.half_length_m - run),
                      top_width_m=2 * (cfg.half_width_m - run))
    return result
