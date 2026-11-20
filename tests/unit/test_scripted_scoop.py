"""Material accounting and controller contracts, without a physics runtime."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from wheel_loader.control.scripted_scoop import ScriptedScoop, attack_lengths
from wheel_loader.evaluation.material_accounting import (
    AcquisitionFunnel,
    MaterialAccounting,
    inside_bucket,
)
from wheel_loader.evaluation.pile_geometry import sampled_swept_volume
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


def test_physical_volume_drives_mass_and_budget_does_not_change_release():
    kwargs = {
        "height_m": 1.2,
        "width_m": 3.8,
        "depth_m": 5.0,
        "crest_length_m": 1.0,
        "slope_deg": np.rad2deg(np.arctan(0.6)),
        "radius_m": 0.12,
        "particle_density_kg_m3": 2590,
        "target_bulk_density_kg_m3": 1036,
        "target_packing_fraction": 0.4,
    }
    p = ScoopPile(**kwargs, max_particles=1500)
    xyz, radii = p.generate()
    assert 750 < len(radii) < 900
    mass = 4 / 3 * np.pi * radii**3 * 2590
    assert abs(mass.sum() - p.volume_m3 * 1036) <= mass.max() / 2
    assert np.all(radii >= 0.95 * 0.12) and np.all(radii <= 0.12)
    xyz2, radii2 = ScoopPile(**kwargs, max_particles=12000).generate()
    assert np.array_equal(xyz, xyz2) and np.array_equal(radii, radii2)
    # No overlapping elements in the selected release.
    for i in range(len(radii)):
        assert np.all(
            np.linalg.norm(xyz[i + 1 :] - xyz[i], axis=1) > radii[i + 1 :] + radii[i]
        )
    with pytest.raises(ValueError, match="budget"):
        ScoopPile(**kwargs, max_particles=100).generate()


def test_inconsistent_geometry_or_bulk_density_fails():
    with pytest.raises(ValueError, match="inconsistent"):
        ScoopPile(depth_m=100).generate()
    with pytest.raises(ValueError, match="Bulk density"):
        ScoopPile(
            target_bulk_density_kg_m3=1000, target_packing_fraction=0.4
        ).generate()


def test_contact_is_not_acquisition_and_wall_exit_is_not_physical_spillage():
    p = {
        "bucket": {"profile": [[0, 0], [1, 0], [1, 1], [0, 1]]},
        "vehicle": {"bucket_width": 2},
    }
    f = AcquisitionFunnel([2.0, 3.0], p)
    f.sample(np.array([[-0.1, 0, 0.5], [-0.2, 0, 0.5]]), [0, 0], [1, 1], "penetrate")
    assert f.contacted.all() and not f.acquired.any()
    f.sample(np.array([[0.2, 0, 0.5], [-0.2, 0, 0.5]]), [1, 0], [1, 0], "lift_breakout")
    assert f.acquired.tolist() == [True, False]
    f.sample(np.array([[1.1, 0, 0.5], [-0.2, 0, 0.5]]), [0, 0], [0, 0], "reverse")
    assert not f.exited.any()  # A classifier exit through a wall is not a mouth exit.
    f.sample(np.array([[0.2, 0, 0.5], [-0.2, 0, 0.5]]), [1, 0], [0, 0], "reverse")
    f.sample(np.array([[-0.2, 0, 0.5], [-0.2, 0, 0.5]]), [0, 0], [0, 0], "spill_tip")
    assert f.extraction_spilled.tolist() == [True, False]
    assert f.crossings_out == 1


def test_spillage_sequence_uses_cylinders_after_reverse():
    p = load_parameters(P / "configs/vehicle/wa320.yaml")
    cfg = yaml.safe_load((P / "configs/experiments/scripted_scoop.yaml").read_text())
    cfg["controller"]["demonstrate_spillage"] = True
    c = ScriptedScoop(p, cfg["controller"], 10)
    assert c.phases[-4:] == ("reverse", "spill_tip", "spill_hold", "stabilize")
    c.index = c.phases.index("spill_tip")
    u = c.command(0, 0, c.carry, 0)
    assert np.all(abs(u) <= 1)


def test_sweep_measures_union_not_sum_of_cavity_volumes():
    p = {
        "bucket": {"profile": [[0, 0], [1, 0], [1, 1], [0, 1]], "plate_thickness": 0},
        "vehicle": {"bucket_width": 2},
    }
    poses = [(np.zeros(3), np.eye(3)), (np.array([0.5, 0, 0]), np.eye(3))]
    result = sampled_swept_volume(poses, p, cell_m=0.1)
    assert result["volume_m3"] == pytest.approx(3.0)  # 2 + 2 - 1 overlap.


def test_research_attack_provides_one_coarse_diameter_below_planned_edge():
    p = load_parameters(P / "configs/vehicle/wa320.yaml")
    cfg = yaml.safe_load((P / "configs/experiments/scripted_scoop.yaml").read_text())
    case = cfg["scenarios"][cfg["default_scenario"]]
    assert case["attack_lip_height_m"] > 2 * cfg["radius_m"]
    attack_lengths(
        p, cfg["controller"]["attack_angle_deg"], case["attack_lip_height_m"]
    )
