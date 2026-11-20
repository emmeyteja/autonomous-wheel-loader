"""Bounded rectangular grain preparation in metres, independent of the solver."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RectangularBed:
    size_m: tuple = (0.25, 0.28, 0.12)
    radius_m: float = 0.0125
    solid_volume_m3: float = 10 / 2590
    seed: int = 42
    max_particles: int = 2500

    def generate(self):
        size = np.asarray(self.size_m, dtype=float)
        r = self.radius_m
        if (
            size.shape != (3,)
            or not np.isfinite(size).all()
            or np.any(size < 4 * r)
            or not np.isfinite(r)
            or r <= 0
            or not isinstance(self.seed, int)
            or self.seed < 0
        ):
            raise ValueError("Invalid bed dimensions, radius or seed")
        spacing = 2.04 * r
        if np.prod(np.ceil(size / (0.8 * spacing)) + 2) > 20 * self.max_particles:
            raise ValueError("Bed exceeds candidate budget")
        rng = np.random.default_rng(self.seed)
        points = []
        for k, z in enumerate(np.arange(r, size[2] - r, np.sqrt(2 / 3) * spacing)):
            for j, y in enumerate(np.arange(r, size[1] - r, np.sqrt(3) / 2 * spacing)):
                for x in np.arange(r, size[0] - r, spacing):
                    p = np.array(
                        [
                            x + (j % 2) * spacing / 2 + (k % 2) * spacing / 2,
                            y + (k % 2) * spacing * np.sqrt(3) / 6,
                            z,
                        ]
                    )
                    if np.all(p <= size - r):
                        points.append(p)
                        if len(points) > self.max_particles:
                            raise ValueError(
                                "Bed exceeds particle budget; no grains discarded"
                            )
        xyz = np.asarray(points)
        radii = rng.uniform(0.95 * r, r, len(xyz))
        available = np.sum(4 / 3 * np.pi * radii**3)
        if (
            not np.isfinite(self.solid_volume_m3)
            or not 0 < self.solid_volume_m3 <= available
        ):
            raise ValueError("Target mass requires invalid grain inflation")
        radii *= (self.solid_volume_m3 / available) ** (1 / 3)
        # Expand centre separations per axis to match the same outer envelope.
        for axis, extent in enumerate(size):
            p = xyz[:, axis] - np.mean(xyz[:, axis])
            lo, hi = 1.0, extent / np.ptp(p)
            for _ in range(60):
                scale = (lo + hi) / 2
                if np.max(p * scale + radii) - np.min(p * scale - radii) < extent:
                    lo = scale
                else:
                    hi = scale
            xyz[:, axis] = p * ((lo + hi) / 2)
            xyz[:, axis] -= np.min(xyz[:, axis] - radii)
        xyz[:, 0] += 0.04
        xyz[:, 1] -= size[1] / 2
        return xyz, radii


@dataclass(frozen=True)
class ScoopPile:
    """Coarse free-grain trapezoidal release; slope is preparation, not repose.

    Uses the same staggered lattice as the existing material diagnostics. Every
    element is retained; the physics solver, not this generator, settles the pile.
    """

    height_m: float = 0.7
    width_m: float = 2.4
    crest_length_m: float = 0.5
    slope_deg: float = 32.0
    radius_m: float = 0.10
    seed: int = 42
    max_particles: int = 2000
    depth_m: float | None = None
    target_bulk_density_kg_m3: float | None = None
    particle_density_kg_m3: float = 2590.0
    target_packing_fraction: float | None = None

    @property
    def depth(self):
        return self.crest_length_m + 2 * self.height_m / np.tan(
            np.deg2rad(self.slope_deg)
        )

    @property
    def volume_m3(self):
        """Analytic trapezoidal release volume, before gravity settling."""
        return self.width_m * self.height_m * (self.depth + self.crest_length_m) / 2

    def material_design(self):
        """Mass is volume times bulk density; max_particles is only a guard."""
        if self.target_bulk_density_kg_m3 is None:
            if self.target_packing_fraction is not None:
                raise ValueError(
                    "Provide both target bulk density and packing fraction"
                )
            return None  # Exact historical small-debug lattice, not a mass target.
        rho, bulk, packing = (
            self.particle_density_kg_m3,
            self.target_bulk_density_kg_m3,
            self.target_packing_fraction,
        )
        if not all(np.isfinite(v) and v > 0 for v in (rho, bulk)) or not (
            packing is not None and np.isfinite(packing) and 0 < packing < 0.74
        ):
            raise ValueError("Invalid solid/bulk density or target packing fraction")
        if not np.isclose(bulk, rho * packing, rtol=1e-9):
            raise ValueError(
                "Bulk density must equal solid density times packing fraction"
            )
        # Uniform radius distribution on [0.95r, r], retained from the diagnostic.
        mean_solid_volume = (
            4 / 3 * np.pi * self.radius_m**3 * (1 - 0.95**4) / (4 * 0.05)
        )
        mass = self.volume_m3 * bulk
        return {
            "requested_volume_m3": self.volume_m3,
            "target_bulk_density_kg_m3": bulk,
            "target_packing_fraction": packing,
            "target_material_mass_kg": mass,
            "expected_particle_mass_kg": rho * mean_solid_volume,
            "estimated_particle_count": round(mass / (rho * mean_solid_volume)),
        }

    def generate(self):
        if (
            not all(
                np.isfinite(x) and x > 0
                for x in (
                    self.height_m,
                    self.width_m,
                    self.crest_length_m,
                    self.radius_m,
                )
            )
            or not 10 <= self.slope_deg <= 60
        ):
            raise ValueError("Invalid coarse pile dimensions or slope")
        if self.height_m < 4 * self.radius_m or self.width_m < 4 * self.radius_m:
            raise ValueError("Pile must span at least four element radii")
        if (
            not isinstance(self.seed, int)
            or self.seed < 0
            or not isinstance(self.max_particles, int)
            or self.max_particles < 1
        ):
            raise ValueError("Invalid seed or particle budget")
        slope = np.tan(np.deg2rad(self.slope_deg))
        length = self.crest_length_m + 2 * self.height_m / slope
        if self.depth_m is not None and (
            not np.isfinite(self.depth_m)
            or not np.isclose(self.depth_m, length, rtol=1e-6, atol=1e-6)
        ):
            raise ValueError(
                "Depth, height, crest and slope describe inconsistent trapezoid geometry"
            )
        design = self.material_design()
        if design and design["estimated_particle_count"] > self.max_particles:
            raise ValueError(
                f"Requested volume/density requires approximately {design['estimated_particle_count']} "
                f"particles; CPU particle budget is {self.max_particles}; no mass discarded"
            )
        spacing = 2.04 * self.radius_m
        if (
            np.prod(
                np.ceil(
                    np.array([length, self.width_m, self.height_m]) / (0.8 * spacing)
                )
                + 2
            )
            > 20 * self.max_particles
        ):
            raise ValueError("Pile exceeds candidate budget")
        rng = np.random.default_rng(self.seed)
        points = []
        for k, z in enumerate(
            np.arange(self.radius_m, self.height_m, np.sqrt(2 / 3) * spacing)
        ):
            for j, y in enumerate(
                np.arange(
                    -self.width_m / 2 + self.radius_m,
                    self.width_m / 2 - self.radius_m,
                    np.sqrt(3) / 2 * spacing,
                )
            ):
                for x in np.arange(self.radius_m, length - self.radius_m, spacing):
                    xx = x + (j % 2 + k % 2) * spacing / 2
                    yy = y + (k % 2) * spacing * np.sqrt(3) / 6
                    ceiling = min(self.height_m, xx * slope, (length - xx) * slope)
                    if (
                        z + self.radius_m <= ceiling
                        and abs(yy) + self.radius_m <= self.width_m / 2
                    ):
                        points.append([xx, yy, z])
                        if design is None and len(points) > self.max_particles:
                            raise ValueError("Pile exceeds budget; no grains discarded")
        if not points:
            raise ValueError("Empty coarse pile")
        xyz = np.asarray(points) + rng.uniform(
            -0.002 * self.radius_m, 0.002 * self.radius_m, (len(points), 3)
        )
        radii = rng.uniform(0.95 * self.radius_m, self.radius_m, len(points))
        if design:
            # Select a deterministic, non-overlapping release with the requested
            # mass. Never shrink radii, inflate density, or truncate to the cap.
            order = rng.permutation(len(points))
            mass = 4 / 3 * np.pi * radii[order] ** 3 * self.particle_density_kg_m3
            cumulative = np.r_[0.0, np.cumsum(mass)]
            target = design["target_material_mass_kg"]
            if cumulative[-1] < target:
                raise ValueError(
                    "Requested packing exceeds non-overlapping release capacity; "
                    "enlarge the geometry or explicitly change the packing experiment"
                )
            count = int(np.argmin(abs(cumulative - target)))
            if count < 1 or count > self.max_particles:
                raise ValueError(
                    "Physical target exceeds particle budget; no mass discarded"
                )
            chosen = np.sort(order[:count])
            xyz, radii = xyz[chosen], radii[chosen]
        return xyz, radii
