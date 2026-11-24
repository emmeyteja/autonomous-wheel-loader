"""Versioned material provenance and numerical resolution, without a solver adapter.

Loading these records never enables granular terrain or changes the M2 reference.
Physical PSD quantiles are distinct from numerical element diameters.
"""

import hashlib
import json
import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from pathlib import Path
from types import MappingProxyType

import yaml

PROPERTY_UNITS = {
    "bulk": {
        "bulk_density": "kg/m^3",
        "particle_density": "kg/m^3",
        "packing_fraction": "1",
        "internal_friction_angle": "rad",
        "cohesion": "Pa",
        "dilatancy_angle": "rad",
        "bulk_strength": "Pa",
        "shear_strength": "Pa",
        "young_modulus": "Pa",
    },
    "particle": {
        "d10": "m",
        "d50": "m",
        "d90": "m",
        "psd_basis": "category",
        "shape": "category",
        "particle_particle_friction": "1",
        "particle_bucket_friction": "1",
        "rolling_resistance": "m",  # Physical rolling torque / normal force; test conditions required.
        "restitution": "1",
        "contact_stiffness": "N/m",
        "contact_damping": "N*s/m",
    },
    "environment": {"moisture_content": "1", "temperature": "K"},
}
STATUSES = {
    "unknown",
    "reported",
    "measured",
    "estimated",
    "calibrated",
    "derived",
    "assumed",
}


@dataclass(frozen=True)
class MaterialProperty:
    value: float | str | None
    unit: str
    status: str
    source: str
    calibration_dataset: str | None
    notes: str

    def __post_init__(self):
        if self.status not in STATUSES or any(
            not isinstance(v, str) or not v.strip()
            for v in (self.unit, self.source, self.notes)
        ):
            raise ValueError("Property needs status, units, source and interpretation")
        if (self.status == "unknown") != (self.value is None):
            raise ValueError("Only unknown properties may have null values")
        if self.value is not None:
            if self.unit == "category":
                if not isinstance(self.value, str) or not self.value.strip():
                    raise ValueError("Category must be nonempty text")
            elif (
                isinstance(self.value, bool)
                or not isinstance(self.value, (int, float))
                or not math.isfinite(self.value)
            ):
                raise ValueError("Numeric properties must be finite numbers")
        if self.status == "calibrated" and not self.calibration_dataset:
            raise ValueError("Calibrated properties require a dataset reference")
        if self.calibration_dataset is not None and (
            not isinstance(self.calibration_dataset, str)
            or not self.calibration_dataset.strip()
        ):
            raise ValueError("Dataset reference must be nonempty text or null")


@dataclass(frozen=True)
class MaterialSpecification:
    material_id: str
    description: str
    properties: Mapping[str, MaterialProperty]

    def __post_init__(self):
        object.__setattr__(self, "properties", MappingProxyType(dict(self.properties)))

    @property
    def fingerprint(self):
        """Provenance-inclusive identity, independent of backend/resolution."""
        record = {
            "material_id": self.material_id,
            "description": self.description,
            "properties": {k: asdict(v) for k, v in sorted(self.properties.items())},
        }
        return hashlib.sha256(
            json.dumps(record, sort_keys=True, allow_nan=False).encode()
        ).hexdigest()

    def require_values(self, keys):
        """Availability check only; known is not the same as validated/calibrated."""
        unknown = [k for k in keys if self.properties[k].value is None]
        if unknown:
            raise ValueError("Unknown physical properties: " + ", ".join(unknown))
        return tuple(self.properties[k].value for k in keys)


def load_material(path):
    raw = yaml.safe_load(Path(path).read_text())
    expected = {"schema_version", "material_id", "description", *PROPERTY_UNITS}
    if (
        not isinstance(raw, dict)
        or set(raw) != expected
        or type(raw["schema_version"]) is not int
        or raw["schema_version"] != 1
    ):
        raise ValueError(
            "Expected material schema version 1 and explicit physical groups"
        )
    if any(
        not isinstance(raw[k], str) or not raw[k].strip()
        for k in ("material_id", "description")
    ):
        raise ValueError("Material ID and description are required")
    properties = {}
    for group, units in PROPERTY_UNITS.items():
        if not isinstance(raw[group], dict) or set(raw[group]) != set(units):
            raise ValueError("Unexpected/missing physical fields: " + group)
        for key, unit in units.items():
            try:
                p = MaterialProperty(**raw[group][key])
            except (TypeError, KeyError) as exc:
                raise ValueError(
                    "Invalid property record: " + group + "." + key
                ) from exc
            if p.unit != unit:
                raise ValueError("Unit mismatch: " + group + "." + key)
            if isinstance(p.value, (int, float)):
                if (p.value < 0 and key != "dilatancy_angle") or (
                    key
                    in {
                        "bulk_density",
                        "particle_density",
                        "packing_fraction",
                        "d10",
                        "d50",
                        "d90",
                        "temperature",
                        "young_modulus",
                    }
                    and p.value == 0
                ):
                    raise ValueError("Invalid physical range: " + key)
                if (
                    key in {"packing_fraction", "restitution", "moisture_content"}
                    and p.value > 1
                ):
                    raise ValueError("Fraction exceeds one: " + key)
                if (
                    key in {"internal_friction_angle", "dilatancy_angle"}
                    and abs(p.value) >= math.pi / 2
                ):
                    raise ValueError("Angle must be below pi/2 radians")
            properties[group + "." + key] = p
    grading = [properties["particle." + k].value for k in ("d10", "d50", "d90")]
    known = [v for v in grading if v is not None]
    if known != sorted(known):
        raise ValueError("Physical PSD quantiles must be ordered")
    basis = properties["particle.psd_basis"].value
    if basis is not None and basis not in {"mass", "volume", "number"}:
        raise ValueError("PSD basis must be mass, volume or number")
    if known and basis is None:
        raise ValueError("Known physical PSD quantiles require a weighting basis")
    bulk, solid, packing = [
        properties["bulk." + k].value
        for k in ("bulk_density", "particle_density", "packing_fraction")
    ]
    if bulk is not None and solid is not None and bulk > solid:
        raise ValueError("Bulk density exceeds particle density")
    if None not in (bulk, solid, packing) and not math.isclose(
        bulk / solid, packing, rel_tol=1e-6
    ):
        raise ValueError("Packing fraction conflicts with recorded density ratio")
    return MaterialSpecification(raw["material_id"], raw["description"], properties)


@dataclass(frozen=True)
class NumericalResolution:
    """Numerical choices, never a physical material or a calibrated contact law."""

    schema_version: int
    resolution_id: str
    backend: str
    representation: str
    dem_diameter_range_m: tuple[float, float] | None
    continuum_cell_size_m: float | None
    active_zone_particle_diameter_m: float | None
    physics_dt_s: float
    control_dt_s: float

    def __post_init__(self):
        if (
            type(self.schema_version) is not int
            or self.schema_version != 1
            or any(
                not isinstance(v, str) or not v.strip()
                for v in (self.resolution_id, self.backend)
            )
        ):
            raise ValueError("Invalid numerical resolution identity")
        if self.representation not in {"full_dem", "multiscale"}:
            raise ValueError("Unknown numerical representation")
        if self.dem_diameter_range_m is not None:
            diameter = tuple(self.dem_diameter_range_m)
            object.__setattr__(self, "dem_diameter_range_m", diameter)
            if (
                len(diameter) != 2
                or not all(
                    not isinstance(v, bool)
                    and isinstance(v, (int, float))
                    and math.isfinite(v)
                    and v > 0
                    for v in diameter
                )
                or diameter[0] > diameter[1]
            ):
                raise ValueError("Invalid numerical DEM diameters")
        if self.physics_dt_s is None or self.control_dt_s is None:
            raise ValueError("Explicit physics/control periods are required")
        for v in (
            self.physics_dt_s,
            self.control_dt_s,
            self.continuum_cell_size_m,
            self.active_zone_particle_diameter_m,
        ):
            if v is not None and (
                isinstance(v, bool)
                or not isinstance(v, (int, float))
                or not math.isfinite(v)
                or v <= 0
            ):
                raise ValueError(
                    "Numerical lengths/timesteps must be positive and finite"
                )
        if (
            not math.isclose(
                self.control_dt_s / self.physics_dt_s,
                round(self.control_dt_s / self.physics_dt_s),
                abs_tol=1e-8,
            )
            or self.control_dt_s < self.physics_dt_s
        ):
            raise ValueError(
                "Control period must contain an integral number of physics steps"
            )
        if self.representation == "full_dem" and (
            self.dem_diameter_range_m is None
            or self.continuum_cell_size_m is not None
            or self.active_zone_particle_diameter_m is not None
        ):
            raise ValueError(
                "Full DEM needs element sizes and no implicit continuous/active zone"
            )
        if self.representation == "multiscale" and (
            self.continuum_cell_size_m is None
            or self.active_zone_particle_diameter_m is None
        ):
            raise ValueError(
                "Multiscale needs separate continuum and active-zone resolutions"
            )

    @property
    def fingerprint(self):
        return hashlib.sha256(
            json.dumps(asdict(self), sort_keys=True, allow_nan=False).encode()
        ).hexdigest()


def load_resolution(path):
    try:
        return NumericalResolution(**yaml.safe_load(Path(path).read_text()))
    except TypeError as exc:
        raise ValueError("Invalid numerical resolution record") from exc
