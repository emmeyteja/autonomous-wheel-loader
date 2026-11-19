"""Pure sensor geometry and coarse observed-surface height mapping."""

import numpy as np


def ray_directions(azimuth_deg, elevation_deg, columns, channels):
    if columns < 2 or channels < 2:
        raise ValueError("Need at least two ray rows/columns")
    a, e = np.meshgrid(
        np.deg2rad(np.linspace(*azimuth_deg, columns)),
        np.deg2rad(np.linspace(*elevation_deg, channels)),
    )
    return np.column_stack(
        (
            np.cos(e).ravel() * np.cos(a).ravel(),
            np.cos(e).ravel() * np.sin(a).ravel(),
            np.sin(e).ravel(),
        )
    )


def rotation_rpy_deg(rpy):
    r, p, y = np.deg2rad(rpy)
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return (
        np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    )


def map_points(
    points_sensor,
    pose_rear,
    extrinsic_translation,
    extrinsic_rotation,
    halfbase,
    root_height,
):
    """[B,R,3] points -> odom frame, using estimated planar rear pose only.

    Assumes a flat world, fixed root height, equal axle offsets; no simulator pose input.
    """
    points = np.asarray(points_sensor)
    pose = np.asarray(pose_rear)
    offset = np.asarray(extrinsic_translation) + [halfbase, 0, root_height]
    body = points @ np.asarray(extrinsic_rotation).T + offset
    c, s = np.cos(pose[:, 2]), np.sin(pose[:, 2])
    out = np.empty_like(body)
    out[:, :, 0] = (
        pose[:, None, 0] + c[:, None] * body[:, :, 0] - s[:, None] * body[:, :, 1]
    )
    out[:, :, 1] = (
        pose[:, None, 1] + s[:, None] * body[:, :, 0] + c[:, None] * body[:, :, 1]
    )
    out[:, :, 2] = body[:, :, 2]
    return out


class HeightGrid:
    def __init__(self, x_bounds, y_bounds, cell_size):
        self.lo = np.array([x_bounds[0], y_bounds[0]], float)
        self.hi = np.array([x_bounds[1], y_bounds[1]], float)
        self.cell = cell_size
        if cell_size <= 0 or np.any(self.hi <= self.lo):
            raise ValueError("Invalid map bounds")
        self.shape = tuple(np.ceil((self.hi - self.lo) / cell_size).astype(int))
        self.height = np.full(self.shape, np.nan)
        self.count = np.zeros(self.shape, dtype=np.int64)

    def update(self, points):
        points = np.asarray(points).reshape(-1, 3)
        valid = (
            np.isfinite(points).all(axis=1)
            & (points[:, :2] >= self.lo).all(axis=1)
            & (points[:, :2] < self.hi).all(axis=1)
        )
        xyz = points[valid]
        ij = np.floor((xyz[:, :2] - self.lo) / self.cell).astype(int)
        if len(xyz):
            current = np.nan_to_num(self.height, nan=-np.inf)
            np.maximum.at(current, (ij[:, 0], ij[:, 1]), xyz[:, 2])
            self.height[:] = np.where(np.isfinite(current), current, np.nan)
            np.add.at(self.count, (ij[:, 0], ij[:, 1]), 1)
        return len(xyz)
