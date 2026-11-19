"""Sensor units/frames, articulated kinematics, reset isolation and unavailable data."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from wheel_loader.evaluation.measurement_audit import audit_parameters
from wheel_loader.sensing.contracts import Availability, RangeScan
from wheel_loader.sensing.geometry import (
    HeightGrid,
    map_points,
    ray_directions,
    rotation_rpy_deg,
)
from wheel_loader.sensing.hydraulics import gross_cylinder_force_N
from wheel_loader.sensing.noise import RangeNoise
from wheel_loader.sensing.odometry import ArticulatedOdometry

ROOT = Path(__file__).resolve().parents[2]


def test_scan_owns_data_and_requires_nan_for_missing_returns():
    ranges = np.array([[2.0, np.nan]])
    mask = np.array([[True, False]])
    dirs = np.array([[1, 0, 0], [0, 1, 0]])
    scan = RangeScan(np.array([1.0]), np.array([1.02]), ranges, mask, dirs)
    ranges[0, 0] = 4
    assert scan.ranges_m[0, 0] == 2
    assert not scan.ranges_m.flags.writeable
    np.testing.assert_allclose(scan.points_sensor_m()[0, 0], [2, 0, 0])
    with pytest.raises(ValueError):
        RangeScan(np.array([1.0]), np.array([0.5]), ranges, mask, dirs)
    with pytest.raises(ValueError):
        RangeScan(np.array([1.0]), np.array([2.0]), np.zeros((1, 2)), mask, dirs)
    with pytest.raises(ValueError):
        Availability(False, "pressure", "Pa", "none")


def test_articulated_odometry_exact_constant_turn_and_reverse():
    radius, wheelbase, steer = 0.7, 3.0, 0.5
    odom = ArticulatedOdometry(1, radius, wheelbase)
    odom.update(np.zeros((1, 2)), np.array([steer]))
    pose = odom.update(np.ones((1, 2)), np.array([steer]))[0]
    turn_radius = (wheelbase / 2) / np.tan(steer / 2)
    yaw = radius / turn_radius
    np.testing.assert_allclose(
        pose,
        [turn_radius * np.sin(yaw), turn_radius * (1 - np.cos(yaw)), yaw],
        atol=1e-12,
    )
    np.testing.assert_allclose(
        odom.update(np.zeros((1, 2)), np.array([steer])), 0, atol=1e-12
    )


def test_odometry_zero_drive_steering_and_batch_reset_isolation():
    odom = ArticulatedOdometry(2, 0.7, 3.0)
    odom.update(np.zeros((2, 2)), np.zeros(2))
    pose = odom.update(np.ones((2, 2)), np.zeros(2))
    np.testing.assert_allclose(pose, [[0.7, 0, 0], [0.7, 0, 0]])
    odom.reset([True, False])
    pose = odom.update(2 * np.ones((2, 2)), np.array([0, 0.1]))
    np.testing.assert_array_equal(pose[0], np.zeros(3))
    assert pose[1, 0] > 0.7 and pose[1, 2] != 0
    with pytest.raises(ValueError):
        odom.update(20 * np.ones((2, 2)), np.zeros(2))


def test_range_noise_reset_does_not_change_other_environment_sequence():
    args = (2, 42, 0.02, 0.1, 0.3, 25)
    a, b = RangeNoise(*args), RangeNoise(*args)
    ranges = np.full((2, 100), 5.0)
    first, _ = a.apply(ranges)
    b.apply(ranges)
    a.reset([True, False])
    again, _ = a.apply(ranges)
    control, _ = b.apply(ranges)
    np.testing.assert_allclose(again[0], first[0], equal_nan=True)
    np.testing.assert_allclose(again[1], control[1], equal_nan=True)
    assert not np.array_equal(first[0], first[1], equal_nan=True)


def test_lidar_geometry_and_map_unknown_cells_remain_nan():
    dirs = ray_directions([-30, 30], [-10, 10], 3, 2)
    np.testing.assert_allclose(np.linalg.norm(dirs, axis=1), 1)
    # Sensor +X is body +Y under 90 degree yaw; rear pose yaw also +90 degrees.
    points = map_points(
        np.array([[[2.0, 0, 0]]]),
        np.array([[10.0, 20.0, np.pi / 2]]),
        [0, 0, 1],
        rotation_rpy_deg([0, 0, 90]),
        1.5,
        1.0,
    )
    np.testing.assert_allclose(points[0, 0], [8, 21.5, 2], atol=1e-12)
    grid = HeightGrid([0, 2], [0, 2], 1)
    grid.update([[0.1, 0.1, 0.5], [0.2, 0.2, 1.2], [np.nan, 0, 0], [3, 3, 3]])
    assert grid.height[0, 0] == 1.2 and grid.count[0, 0] == 2
    assert np.isnan(grid.height[1, 1]) and grid.count[1, 1] == 0


def test_cylinder_force_uses_both_chambers_and_rejects_unknown_areas():
    assert gross_cylinder_force_N(np.array([2e6]), np.array([0.5e6]), 0.1, 0.05)[
        0
    ] == pytest.approx(12762.7201552)
    with pytest.raises(ValueError):
        gross_cylinder_force_N(2e6, 0.5e6, np.nan, 0.05)
    with pytest.raises(ValueError):
        gross_cylinder_force_N(2e6, np.nan, 0.1, 0.05)


def test_measurement_ledger_covers_all_active_parameters_and_no_false_accuracy():
    p = yaml.safe_load((ROOT / "configs/vehicle/wa320.yaml").read_text())
    ledger = yaml.safe_load((ROOT / "configs/measurements/wa320_7.yaml").read_text())
    result = audit_parameters(p, ledger)
    assert result["active"]["vehicle.mass"]["status"] == "published_nominal"
    assert result["active"]["boom.pivot"]["status"] == "engineering_estimate"
    assert result["active"]["solver.dt"]["status"] == "numerical_setting"
    assert all(r["true_vehicle_error"] is None for r in result["active"].values())
    assert not result["real_machine_measurements"]["available"]
    assert not result["physical_accuracy_established"]
    p["boom"]["new_dimension"] = 1
    with pytest.raises(ValueError, match="coverage"):
        audit_parameters(p, ledger)
