"""Normalized input, first-order integration, saturation and transmission checks."""

from pathlib import Path

import numpy as np
import pytest

from wheel_loader.control.cylinder_actuation import (
    FirstOrderCylinders,
    cylinder_geometry,
)
from wheel_loader.vehicle.parameters import load_parameters

ROOT = Path(__file__).resolve().parents[1]


def test_exact_first_order_step_limits_force_and_masked_reset():
    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    a = FirstOrderCylinders(p, [0, 0.15, -0.05], count=2)
    initial = a.x.copy()
    u = np.ones((2, 4))
    dt = 0.03
    force, _ = a.step(u, dt, initial, np.zeros_like(initial))
    expected_v = a.vmax * (1 - np.exp(-dt / a.tau))
    expected_x = initial + a.vmax * dt - a.vmax * a.tau * (1 - np.exp(-dt / a.tau))
    np.testing.assert_allclose(a.v, np.tile(expected_v, (2, 1)))
    np.testing.assert_allclose(a.x, expected_x)
    assert (abs(force) <= a.fmax).all()
    for _ in range(1000):
        a.step(u, 0.1, initial, np.zeros_like(initial))
    np.testing.assert_allclose(a.x, np.tile(a.hi, (2, 1)))
    assert (a.v == 0).all()
    other = a.x[1].copy()
    a.reset([True, False])
    np.testing.assert_allclose(a.x[0], initial[0])
    np.testing.assert_allclose(a.x[1], other)
    with pytest.raises(ValueError):
        a.step(np.full((2, 4), 1.01), dt, initial, np.zeros_like(initial))


def test_cylinder_jacobians_match_finite_difference_and_manual_strokes():
    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    angles = np.array([0.2, 0.6, 0.1])
    _length, jac, _ = cylinder_geometry(p, angles)
    for k in range(3):
        d = np.eye(3)[k] * 1e-6
        difference = (
            cylinder_geometry(p, angles + d)[0] - cylinder_geometry(p, angles - d)[0]
        ) / 2e-6
        np.testing.assert_allclose(jac[:, k], difference, atol=1e-8)
    for angle, key in [
        (p["articulation"]["min_angle"], "pin_distance_max"),
        (p["articulation"]["max_angle"], "pin_distance_min"),
    ]:
        assert cylinder_geometry(p, [angle, 0, 0])[0][0] == pytest.approx(
            p["cylinders"]["steering"][key], abs=1e-10
        )
    assert p["cylinders"]["lift"]["bore"] == 0.120
    assert p["cylinders"]["lift"]["rod_diameter"] == 0.075
    assert p["cylinders"]["lift"]["stroke"] == 0.729
    assert p["cylinders"]["bucket"]["stroke"] == 0.558


def test_integral_hold_force_is_bounded_and_reset_per_environment():
    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    a = FirstOrderCylinders(p, [0, 0.15, -0.05], count=2)
    measured = a.x - 0.001
    first, _ = a.step(np.zeros((2, 4)), 0.01, measured, np.zeros_like(measured))
    for _ in range(100):
        last, _ = a.step(np.zeros((2, 4)), 0.01, measured, np.zeros_like(measured))
    assert (last > first).all()
    assert (abs(last) <= a.fmax).all()
    retained = a.integral_force[1].copy()
    a.reset([True, False])
    assert (a.integral_force[0] == 0).all()
    np.testing.assert_array_equal(a.integral_force[1], retained)


def test_physical_actuation_applies_only_linear_cylinder_and_wheel_efforts():
    from wheel_loader.control.cylinder_actuation import joint_efforts
    from wheel_loader.vehicle.kinematics import passive_joint_positions

    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    state = passive_joint_positions(p, 0.15, -0.05) | {
        "articulation_joint": 0.0,
        **{
            f"{a}_{s}_wheel_joint": 0.0
            for a in ("front", "rear")
            for s in ("left", "right")
        },
    }
    names = list(state)
    q = np.array(list(state.values()))
    qd = np.zeros_like(q)
    actuator = FirstOrderCylinders(p, [0, 0.15, -0.05])
    effort, meta = joint_efforts(p, actuator, [0, 0.2, 0.3, 0.4], q, qd, names, 0.03)
    for name in (
        "articulation_joint",
        "boom_joint",
        "tilt_joint",
        "rod_joint",
        "bucket_joint",
    ):
        assert effort[names.index(name)] == 0
    for name, spec in p["cylinder_bodies"].items():
        assert effort[names.index(name + "_hinge_joint")] == 0
        assert (
            abs(effort[names.index(name + "_slider_joint")])
            <= p["actuators"][spec["kind"]]["fmax"]
        )
    assert meta["force_N"][1] != 0
