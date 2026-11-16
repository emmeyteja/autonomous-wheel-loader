"""Explicit model selection. Archive entries are metadata, never runtime dependencies."""

from dataclasses import dataclass
from pathlib import Path

from .parameters import load_parameters


@dataclass(frozen=True)
class WA320Loader:
    parameters: dict

    @classmethod
    def load(cls, root: Path):
        return cls(load_parameters(root / "configs/vehicle/wa320.yaml"))


@dataclass(frozen=True)
class KomodoReference:
    path: str = "extras/komodo"
    active: bool = False


VEHICLE_MODELS = {"wa320": WA320Loader, "komodo": KomodoReference}


def select_vehicle(name="wa320", *, root=Path("."), allow_reference=False):
    if name not in VEHICLE_MODELS:
        raise ValueError(f"Unknown vehicle: {name}")
    if name == "komodo":
        if not allow_reference:
            raise ValueError(
                "Komodo is archived. Explicit reference opt-in is required."
            )
        return KomodoReference()
    return WA320Loader.load(Path(root))
