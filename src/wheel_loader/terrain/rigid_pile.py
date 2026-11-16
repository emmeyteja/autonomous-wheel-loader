"""Rigid trapezoidal prism for collision/approach checks, never excavatable soil."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RigidPile:
    length: float = 4.0
    width: float = 5.0
    height: float = 1.5
    crest_length: float = 1.5

    def vertices(self):
        if (
            not all(
                math.isfinite(v) and v > 0
                for v in (self.length, self.width, self.height, self.crest_length)
            )
            or self.crest_length >= self.length
        ):
            raise ValueError(
                "Rigid pile requires finite positive dimensions and a narrower crest"
            )
        return np.array(
            [
                (x, y, z)
                for y in (-self.width / 2, self.width / 2)
                for x, z in [
                    (-self.length / 2, 0),
                    (self.length / 2, 0),
                    (self.crest_length / 2, self.height),
                    (-self.crest_length / 2, self.height),
                ]
            ]
        )


TERRAIN_BACKENDS = {
    "rigid_pile": RigidPile,
    "granular_dem": None,
    "multiscale_dem": None,
    "heightmap": None,
}


def select_terrain(name="rigid_pile", **parameters):
    if name not in TERRAIN_BACKENDS:
        raise ValueError(f"Unknown terrain: {name}")
    backend = TERRAIN_BACKENDS[name]
    if backend is None:
        raise NotImplementedError(f"{name} is not implemented or calibrated")
    result = backend(**parameters)
    result.vertices()
    return result


def load_terrain(path):
    """Read the same terrain configuration used by the optional simulator scene."""
    from pathlib import Path

    import yaml

    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or "backend" not in data:
        raise ValueError("Terrain configuration needs an explicit backend")
    backend = data.pop("backend")
    return select_terrain(backend, **data)
