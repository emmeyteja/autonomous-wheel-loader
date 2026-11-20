"""Measured coarse-pile envelopes and sampled bucket-cavity swept volume (SI)."""

import numpy as np

from wheel_loader.evaluation.material_accounting import bucket_payload_mask


def settled_envelope(points, radii, mass, density):
    """Convex envelope includes interparticle voids, not just solid volume.

    Sphere surfaces use 26 directions. This is an explicitly approximate,
    convex-envelope bulk volume; concave pile cavities are included.
    """
    from scipy.spatial import ConvexHull

    directions = np.array(
        [
            [x, y, z]
            for x in (-1, 0, 1)
            for y in (-1, 0, 1)
            for z in (-1, 0, 1)
            if x or y or z
        ],
        float,
    )
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    surface = points[:, None, :] + radii[:, None, None] * directions
    volume = float(ConvexHull(surface.reshape(-1, 3)).volume)
    return {
        "volume_m3": volume,
        "material_mass_kg": float(mass),
        "effective_bulk_density_kg_m3": float(mass / volume),
        "effective_packing_fraction": float(mass / density / volume),
        "method": "convex hull of 26 surface samples per sphere; includes voids and concavities; not exact sand bulk volume",
    }


def sampled_swept_volume(bucket_poses, parameters, cell_m=0.10):
    """Union of measured cavity poses, sampled on a fixed world voxel grid.

    Only penetration/curl-advance poses are supplied by the caller. This is a
    discretized cavity-sweep diagnostic, not a measurement of excavated solids.
    """
    if not bucket_poses:
        return None
    polygon = np.asarray(parameters["bucket"]["profile"])
    half = parameters["vehicle"]["bucket_width"] / 2
    vertices = np.array([[x, y, z] for x, z in polygon for y in (-half, half)])
    world = np.concatenate([vertices @ r.T + x for x, r in bucket_poses])
    lo, hi = world.min(axis=0), world.max(axis=0)
    dimensions = np.ceil((hi - lo) / cell_m).astype(int)
    if np.prod(dimensions) > 2_000_000:
        raise ValueError(
            "Swept-volume diagnostic exceeds voxel budget; explicitly coarsen measurement"
        )
    grid = np.stack(
        np.meshgrid(
            *[lo[i] + (np.arange(n) + 0.5) * cell_m for i, n in enumerate(dimensions)],
            indexing="ij",
        ),
        axis=-1,
    ).reshape(-1, 3)
    occupied = np.zeros(len(grid), bool)
    for x, r in bucket_poses:
        remaining = np.flatnonzero(~occupied)
        occupied[remaining] |= bucket_payload_mask(
            (grid[remaining] - x) @ r, parameters
        )
    return {
        "volume_m3": float(occupied.sum() * cell_m**3),
        "voxel_edge_m": cell_m,
        "sampled_pose_count": len(bucket_poses),
        "method": "voxel union of measured bucket centroid-classifier cavities during penetration/curl advance; 20 Hz poses; approximate, not excavated volume",
    }
