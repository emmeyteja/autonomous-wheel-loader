from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from wheel_loader.terrain.materials import (
    load_material,
    load_resolution,
)

ROOT = Path(__file__).resolve().parents[2]
PROFILES = ROOT / "configs/terrain/materials/v1"


def write_profile(tmp_path, change):
    raw = yaml.safe_load((PROFILES / "aoshima_reference_gravel.yaml").read_text())
    change(raw)
    path = tmp_path / "material.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def test_profiles_preserve_known_and_unknown_provenance():
    gravel = load_material(PROFILES / "aoshima_reference_gravel.yaml")
    assert gravel.require_values(["bulk.bulk_density"]) == (1727.0,)
    assert gravel.properties["bulk.particle_density"].status == "reported"
    assert gravel.properties["bulk.bulk_density"].status == "measured"
    # 30–40 mm field description is not a reported PSD percentile.
    assert gravel.properties["particle.d50"].value is None
    for name in ["dry_sand", "dry_coal"]:
        material = load_material(PROFILES / (name + ".yaml"))
        with pytest.raises(ValueError, match="Unknown physical"):
            material.require_values(["bulk.bulk_density"])


def test_material_identity_is_independent_of_numerical_resolution():
    gravel = load_material(PROFILES / "aoshima_reference_gravel.yaml")
    before = gravel.fingerprint
    coarse = load_resolution(ROOT / "configs/terrain/resolutions/m2_reference.yaml")
    finer = replace(coarse, dem_diameter_range_m=(0.09, 0.11))
    assert coarse.fingerprint != finer.fingerprint
    assert (
        load_material(PROFILES / "aoshima_reference_gravel.yaml").fingerprint == before
    )
    with pytest.raises(TypeError):
        gravel.properties["bulk.bulk_density"] = None


@pytest.mark.parametrize(
    "change, message",
    [
        (
            lambda r: r["particle"].update(dem_diameter_range_m=[0.2, 0.4]),
            "physical fields",
        ),
        (lambda r: r["bulk"]["bulk_density"].update(unit="g/cm^3"), "Unit mismatch"),
        (lambda r: r["bulk"]["bulk_density"].update(status="unknown"), "null values"),
        (
            lambda r: r["bulk"]["bulk_density"].update(value=float("nan")),
            "finite numbers",
        ),
        (lambda r: r["bulk"]["bulk_density"].update(value=True), "finite numbers"),
        (
            lambda r: r["bulk"]["bulk_density"].update(status="calibrated"),
            "dataset reference",
        ),
        (lambda r: r["bulk"]["packing_fraction"].update(value=0.4), "density ratio"),
    ],
)
def test_invalid_or_ambiguous_material_records_fail(tmp_path, change, message):
    with pytest.raises(ValueError, match=message):
        load_material(write_profile(tmp_path, change))


def test_unordered_physical_psd_fails(tmp_path):
    def change(raw):
        for key, v in [("d10", 0.04), ("d50", 0.03), ("d90", 0.05)]:
            raw["particle"][key].update(value=v, status="estimated")

    with pytest.raises(ValueError, match="quantiles"):
        load_material(write_profile(tmp_path, change))


def test_dilatancy_can_represent_contraction(tmp_path):
    path = write_profile(
        tmp_path, lambda r: r["bulk"]["dilatancy_angle"].update(value=-0.1)
    )
    assert load_material(path).properties["bulk.dilatancy_angle"].value == -0.1


def test_resolution_requires_explicit_active_zone_and_consistent_time():
    reference = load_resolution(ROOT / "configs/terrain/resolutions/m2_reference.yaml")
    with pytest.raises(ValueError, match="Multiscale"):
        replace(reference, representation="multiscale")
    with pytest.raises(ValueError, match="integral"):
        replace(reference, control_dt_s=0.001)
    with pytest.raises(ValueError, match="positive and finite"):
        replace(reference, physics_dt_s=-1.0)
    with pytest.raises(ValueError, match="DEM"):
        replace(reference, dem_diameter_range_m=(0.4, 0.2))


def test_physical_rolling_measurement_and_psd_basis(tmp_path):
    def change(raw):
        raw["particle"]["rolling_resistance"].update(
            value=0.002,
            status="measured",
            notes="Torque divided by 10 N preload, sphere radius 0.01 m, quasi-static bench fixture; illustrative test record only.",
        )

    path = write_profile(tmp_path, change)
    assert load_material(path).require_values(["particle.rolling_resistance"]) == (
        0.002,
    )
    path = write_profile(
        tmp_path,
        lambda r: r["particle"]["rolling_resistance"].update(
            unit="1", value=0.02, status="reported"
        ),
    )
    with pytest.raises(ValueError, match="Unit mismatch"):
        load_material(path)
    path = write_profile(
        tmp_path, lambda r: r["particle"]["d50"].update(value=0.035, status="estimated")
    )
    with pytest.raises(ValueError, match="weighting basis"):
        load_material(path)
