"""Frozen references and multi-pose workspace checks for the v1 approximation."""

from pathlib import Path

import numpy as np
import pytest

from wheel_loader.vehicle.kinematics import rotate, solve_linkage
from wheel_loader.vehicle.parameters import load_parameters

ROOT = Path(__file__).resolve().parents[1]


def test_frozen_reference_configuration_and_v0_baseline():
    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    expected = {
        "mass": 15175.0,
        "wheelbase": 3.030,
        "track_width": 2.050,
        "overall_width": 2.590,
        "ground_clearance": 0.425,
        "bucket_width": 2.685,
        "bucket_volume": 3.0,
        "overall_length": 7.720,
        "overall_height": 3.200,
        "hinge_pin_height": 4.005,
        "bucket_max_height": 5.590,
        "dump_clearance": 2.835,
        "dump_reach": 1.165,
    }
    for name, v in expected.items():
        assert p["vehicle"][name] == pytest.approx(v, abs=1e-12)
    assert np.rad2deg(p["articulation"]["max_angle"]) == pytest.approx(38.5)
    old = load_parameters(ROOT / "configs/vehicle/archive/wa320_estimated_v0.yaml")
    assert old["masses"]["rear_frame"] == 7000 and old["boom"]["length"] == 2.7
    assert "actuators" not in old


def test_multiple_pose_closure_and_published_workspace():
    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    for q, t in p["maneuver"].values():
        pose = solve_linkage(p, q, t)
        assert np.linalg.norm(pose.bucket_anchor - pose.lower_pin) == pytest.approx(
            p["linkage"]["rod_length"], abs=1e-12
        )
    dump = solve_linkage(p, *p["maneuver"]["dump"])
    assert dump.bucket_pin[1] + p["vehicle"]["articulation_height"] == pytest.approx(
        4.005, abs=0.01
    )
    assert dump.lip[1] + p["vehicle"]["articulation_height"] == pytest.approx(
        2.835, abs=0.01
    )
    tirefront = p["vehicle"]["wheelbase"] / 2 + p["vehicle"]["wheel_radius"]
    assert dump.lip[0] - tirefront == pytest.approx(1.165, abs=0.01)
    rollback = solve_linkage(p, p["boom"]["max_angle"], 0.1984564545475358)
    top = rollback.bucket_pin + rotate(
        rollback.boom_elevation + rollback.bucket_angle, p["bucket"]["back_top"]
    )
    assert top[1] + p["vehicle"]["articulation_height"] == pytest.approx(
        5.590, abs=0.02
    )


def test_recorded_workspace_rejects_underlift_even_with_correct_targets():
    from wheel_loader.control.v_cycle import PHASES
    from wheel_loader.evaluation.actuation import check_workspace
    from wheel_loader.vehicle.kinematics import passive_joint_positions

    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    names = ["boom_joint", "tilt_joint", "bucket_joint"]
    states = []
    for phase in ["raise_dump", "dump"]:
        angles = passive_joint_positions(p, *p["maneuver"][phase])
        states.append([angles[n] for n in names])
    recording = {
        "positions": np.asarray(states),
        "joint_names": np.asarray(names),
        "root_poses_xyzw": np.tile(
            [0, 0, p["vehicle"]["articulation_height"], 0, 0, 0, 1], (2, 1)
        ),
        "phase_ids": np.asarray([PHASES.index("raise_dump"), PHASES.index("dump")]),
    }
    assert all(check_workspace(p, recording)["checks"].values())
    recording["positions"][:, 0] -= 0.2
    assert not check_workspace(p, recording)["checks"]["recorded_hinge_height"]
    recording["positions"][0, 0] = float("nan")
    assert not all(check_workspace(p, recording)["checks"].values())
