"""Ground contact settings are explicit, finite and independent of simulator imports."""

from pathlib import Path

import pytest

from wheel_loader.environment.simulation.ground_contact import (
    GroundContactProfile,
    load_ground_profile,
)

ROOT = Path(__file__).resolve().parents[2]


def test_ground_profile_loads():
    assert (
        load_ground_profile(ROOT / "configs/simulation/newton_ground.yaml")
        == GroundContactProfile()
    )


@pytest.mark.parametrize(
    "field,value",
    [
        ("solver", "unknown"),
        ("cone", "unknown"),
        ("tire_geometry", "sphere"),
        ("contact_time_constant", 0),
        ("contact_time_constant", float("nan")),
        ("damping_ratio", float("inf")),
        ("damping_ratio", True),
    ],
)
def test_ground_profile_rejects_invalid_values(field, value):
    with pytest.raises(ValueError):
        GroundContactProfile(**{field: value})
