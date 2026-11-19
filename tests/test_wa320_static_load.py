"""Moment equilibrium and payload load transfer, not manufacturer axle validation."""

from pathlib import Path

import numpy as np
import pytest

from wheel_loader.vehicle.mass_properties import mass_report
from wheel_loader.vehicle.parameters import load_parameters

ROOT = Path(__file__).resolve().parents[1]


def test_empty_transport_axle_equilibrium_and_plausible_support():
    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    r = mass_report(p)
    assert r["front_axle_load_kg"] + r["rear_axle_load_kg"] == pytest.approx(15175)
    half = p["vehicle"]["wheelbase"] / 2
    # Independent moment balance about rear axle.
    assert r["front_axle_load_N"] * (2 * half) == pytest.approx(
        15175 * 9.81 * (r["total_vehicle_COM_m"][0] + half)
    )
    assert 30 < r["front_percentage"] < 60
    assert 40 < r["rear_percentage"] < 70
    assert np.isfinite(r["rear_body_COM_m"]).all()
    assert not r["manufacturer_axle_loads"]


def test_known_payload_changes_axle_reaction_and_unload_recovers_empty():
    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    empty = mass_report(p)
    loaded = mass_report(p, payload_kg=2000, payload_local_m=[0.6, 0, 0.2])
    assert loaded["front_axle_load_kg"] > empty["front_axle_load_kg"] + 2000
    assert loaded["rear_axle_load_kg"] < empty["rear_axle_load_kg"]
    assert loaded["vehicle_mass_kg"] == 17175
    assert mass_report(p) == empty
