"""Material-only release geometry. No simulator dependencies or fixed pile slopes."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Column:
    radius: float = 0.12
    height: float = 0.25
    element_radius: float = 0.0125
    seed: int = 42
    max_particles: int = 2500
    solid_volume_m3: float | None = None

    def generate(self):
        if (
            not all(
                math.isfinite(v) and v > 0
                for v in (self.radius, self.height, self.element_radius)
            )
            or self.radius < 4 * self.element_radius
            or self.height < 4 * self.element_radius
            or not isinstance(self.seed, int)
            or self.seed < 0
            or not isinstance(self.max_particles, int)
            or self.max_particles < 1
        ):
            raise ValueError("Invalid or under-resolved diagnostic column")
        r = self.element_radius
        gap = 0.04 * r
        spacing = 2 * r + gap
        # Refuse oversized requests before allocating candidate arrays.
        candidates = (math.ceil(2 * self.radius / spacing) + 2) ** 2 * (
            math.ceil(self.height / (0.8 * spacing)) + 1
        )
        if candidates > 20 * self.max_particles:
            raise ValueError("Column exceeds candidate budget")
        rng = np.random.default_rng(self.seed)
        points = []
        for k, z in enumerate(
            np.arange(r + gap / 4, self.height - r, math.sqrt(2 / 3) * spacing)
        ):
            for j, y in enumerate(
                np.arange(-self.radius, self.radius, math.sqrt(3) / 2 * spacing)
            ):
                for x in np.arange(-self.radius, self.radius, spacing):
                    xx = x + (j % 2) * spacing / 2 + (k % 2) * spacing / 2
                    yy = y + (k % 2) * spacing * math.sqrt(3) / 6
                    if math.hypot(xx, yy) <= self.radius - r - gap:
                        points.append(
                            np.array([xx, yy, z]) + rng.uniform(-gap / 8, gap / 8, 3)
                        )
                        if len(points) > self.max_particles:
                            raise ValueError(
                                "Column exceeds particle budget; no particles discarded"
                            )
        if not points:
            raise ValueError("Empty column")
        radii = rng.uniform(0.95 * r, r, len(points))
        xyz = np.asarray(points)
        if self.solid_volume_m3 is not None:
            available = float(np.sum(4 / 3 * np.pi * radii**3))
            if not (
                math.isfinite(self.solid_volume_m3)
                and 0 < self.solid_volume_m3 <= available
            ):
                raise ValueError(
                    "Target solid volume must be positive and fit without inflating grains"
                )
            # Preserve the generated size ratios and every element ID. Shrinking
            # radii and expanding centre separations cannot introduce overlap.
            radii *= (self.solid_volume_m3 / available) ** (1 / 3)
            distance = np.linalg.norm(xyz[:, :2], axis=1)
            nonzero = distance > 0
            xy_scale = np.min((self.radius - radii[nonzero]) / distance[nonzero])
            xyz[:, :2] *= xy_scale
            z = xyz[:, 2] - np.mean(xyz[:, 2])
            low, high = 1.0, self.height / np.ptp(z)
            for _ in range(60):
                scale = (low + high) / 2
                extent = np.max(z * scale + radii) - np.min(z * scale - radii)
                if extent < self.height:
                    low = scale
                else:
                    high = scale
            xyz[:, 2] = z * ((low + high) / 2)
            xyz[:, 2] -= np.min(xyz[:, 2] - radii)
        return xyz, radii
