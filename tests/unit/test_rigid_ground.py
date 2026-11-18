"""Ground evaluation rejects spinning wheels, drift and incomplete evidence."""

from pathlib import Path

import numpy as np
import pytest

from wheel_loader.evaluation.rigid_ground import evaluate_ground
from wheel_loader.vehicle.parameters import load_parameters

P = load_parameters(Path(__file__).resolve().parents[2] / "configs/vehicle/wa320.yaml")


def trajectory():
    names = ["articulation_joint", "boom_joint", "tilt_joint", "bucket_joint"] + [
        f"{axle}_{side}_wheel_joint"
        for axle in ("front", "rear")
        for side in ("left", "right")
    ]
    times = np.linspace(0, 15, 151)
    positions = np.zeros((len(times), len(names)))
    positions[:, 4:] = times[:, None]
    roots = np.zeros((len(times), 7))
    roots[:, 0] = times * 0.5
    yaw = np.maximum(0, times - 8) * 0.1
    roots[:, 5] = np.sin(yaw / 2)
    roots[:, 6] = np.cos(yaw / 2)
    gaps = np.full(len(times), 1e-5)
    return times, positions, roots, gaps, names


def test_ground_trajectory_passes():
    assert all(evaluate_ground(P, *trajectory())["checks"].values())


def test_spinning_wheels_does_not_count_as_driving():
    times, q, roots, gaps, names = trajectory()
    roots[:, 0] = 0
    result = evaluate_ground(P, times, q, roots, gaps, names)
    assert result["checks"]["wheel_rotation"]
    assert not result["checks"]["driving"]


def test_drift_limits_and_open_linkage_fail():
    times, q, roots, gaps, names = trajectory()
    roots[:, 1] = times * 0.1
    q[:, 0] = 1
    gaps[:] = 0.01
    result = evaluate_ground(P, times, q, roots, gaps, names)
    for check in ("straight_lateral_drift", "joint_limits", "loop_closure"):
        assert not result["checks"][check]


def test_missing_ground_turn_fails():
    times, q, roots, gaps, names = trajectory()
    roots[:, 5] = 0
    roots[:, 6] = 1
    assert not evaluate_ground(P, times, q, roots, gaps, names)["checks"][
        "ground_turning"
    ]


@pytest.mark.parametrize("fault", ["short", "nonfinite", "quaternion", "timestamps"])
def test_invalid_evidence_rejected(fault):
    times, q, roots, gaps, names = trajectory()
    if fault == "short":
        times, q, roots, gaps = times[:10], q[:10], roots[:10], gaps[:10]
    elif fault == "nonfinite":
        roots[0, 0] = np.nan
    elif fault == "quaternion":
        roots[0, 6] = 0
    else:
        times[1] = times[0]
    with pytest.raises(ValueError):
        evaluate_ground(P, times, q, roots, gaps, names)


def test_force_evidence_rejects_missing_wheels_and_intermittent_support():
    from wheel_loader.evaluation.rigid_ground import evaluate_wheel_support

    times = np.linspace(0, 15, 151)
    forces = np.zeros((len(times), 4, 3))
    forces[:, :, 2] = 1000
    assert evaluate_wheel_support(times, forces)["sustained_wheel_support"]
    forces[50:, 0, 2] = 0
    assert not evaluate_wheel_support(times, forces)["sustained_wheel_support"]
    with pytest.raises(ValueError):
        evaluate_wheel_support(times, forces[:, :3])
    forces[0, 0, 0] = np.nan
    with pytest.raises(ValueError):
        evaluate_wheel_support(times, forces)
