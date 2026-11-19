"""Material accounting and controller contracts, without a physics runtime."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from wheel_loader.control.scripted_scoop import ScriptedScoop, attack_lengths
from wheel_loader.evaluation.material_accounting import (
    MaterialAccounting,
    inside_bucket,
)
from wheel_loader.terrain.bed import ScoopPile
from wheel_loader.vehicle.parameters import load_parameters

P = Path(__file__).resolve().parents[2]


def test_grain_preparation_repeatable_without_overlap():
    pile = ScoopPile()
    xyz, r = pile.generate()
    xyz2, r2 = pile.generate()
    assert np.array_equal(xyz, xyz2) and np.array_equal(r, r2)
    d = np.linalg.norm(xyz[:, None, :] - xyz[None, :, :], axis=-1)
    np.fill_diagonal(d, np.inf)
    assert np.all(d > r[:, None] + r[None, :])
    assert np.all(r > 0)


def test_pile_budget_refuses_truncation():
    with pytest.raises(ValueError, match="budget"):
        ScoopPile(max_particles=1).generate()


def test_exclusive_accounting_acquire_spill_reacquire_and_boundary_loss():
    a = MaterialAccounting([2.0, 3.0, 5.0, 7.0])
    one = a.sample([1, 0, 0, 0], [1, 1, 1, 1], [0, 0, 0, 0])
    assert one["bucket_mass_kg"] == 2 and one["remaining_mass_kg"] == 15
    two = a.sample([0, 1, 0, 0], [1, 1, 0, 0], [0, 0, 0, 1])
    assert two["spilled_mass_kg"] == 2  # Only material that actually left the bucket.
    assert two["remaining_mass_kg"] == 5
    assert two["displaced_unacquired_mass_kg"] == 5
    assert two["bucket_exit_mass_kg"] == 2
    assert two["boundary_loss_mass_kg"] == 7
    assert abs(two["conservation_residual_kg"]) < 1e-12
    three = a.sample([1, 0, 0, 0], [1, 1, 0, 0], [0, 0, 0, 1], delivered=[0, 1, 0, 0])
    assert three["bucket_mass_kg"] == 2 and three["delivered_mass_kg"] == 3
    assert three["spilled_mass_kg"] == 0 and three["conservation_residual_kg"] == 0


def test_missing_ids_rejected():
    with pytest.raises(ValueError, match="IDs"):
        MaterialAccounting([1, 2]).sample([1], [0], [0])


def test_open_cavity_excludes_plates_and_above_mouth():
    polygon = [[0, 0], [1, 0], [1, 1], [0, 1]]
    mask = inside_bucket(
        np.array([[0.5, 0, 0.5], [0.5, 0, 1.1], [0.02, 0, 0.5], [0.5, 0.99, 0.5]]),
        polygon,
        1,
        np.full(4, 0.05),
    )
    assert mask.tolist() == [True, False, False, False]


def test_controller_uses_bounded_normalized_cylinder_commands():
    p = load_parameters(P / "configs/vehicle/wa320.yaml")
    cfg = yaml.safe_load((P / "configs/experiments/scripted_scoop.yaml").read_text())
    c = ScriptedScoop(p, cfg["controller"], cfg["settle_s"])
    for i in range(200):
        u = c.command(i * 0.1, 0, c.transport, 0)
        assert u.shape == (4,) and np.all(abs(u) <= 1)
    assert c.phase == "approach"  # stationary chassis cannot fake penetration


def test_attack_plan_cannot_request_unreachable_pitch():
    p = load_parameters(P / "configs/vehicle/wa320.yaml")
    with pytest.raises(ValueError, match="workspace"):
        attack_lengths(p, 90)


def test_zero_or_negative_controller_limits_rejected():
    p = load_parameters(P / "configs/vehicle/wa320.yaml")
    cfg = yaml.safe_load((P / "configs/experiments/scripted_scoop.yaml").read_text())
    cfg["controller"]["penetration_depth_m"] = -1
    with pytest.raises(ValueError, match="parameters"):
        ScriptedScoop(p, cfg["controller"], 6)


def test_nonmonotonic_or_invalid_observation_rejected():
    p = load_parameters(P / "configs/vehicle/wa320.yaml")
    cfg = yaml.safe_load((P / "configs/experiments/scripted_scoop.yaml").read_text())
    c = ScriptedScoop(p, cfg["controller"], cfg["settle_s"])
    c.command(1, 0, c.transport, 0)
    with pytest.raises(ValueError, match="observation"):
        c.command(0.5, 0, c.transport, 0)
    with pytest.raises(ValueError, match="observation"):
        c.command(2, 0, [float("nan"), 1, 2], 0)
