"""Frame/rate correctness independent of simulator installation."""

import math

import numpy as np

from wheel_loader.environment.simulation.replay_state import (
    rpy_degrees,
    world_angular_velocity,
)


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_world_angular_rate_crosses_yaw_wrap():
    before, after = rotation_z(math.pi - 0.01), rotation_z(-math.pi + 0.01)
    np.testing.assert_allclose(
        world_angular_velocity(before, after, 0.02), [0, 0, 1], atol=1e-10
    )


def test_world_rate_follows_world_axes_not_body_axes():
    # Local X spin after a 90-degree world Z rotation is world Y spin.
    before = rotation_z(math.pi / 2)
    angle = 0.01
    local = np.array(
        [
            [1, 0, 0],
            [0, math.cos(angle), -math.sin(angle)],
            [0, math.sin(angle), math.cos(angle)],
        ]
    )
    np.testing.assert_allclose(
        world_angular_velocity(before, before @ local, 0.01), [0, 1, 0], atol=1e-10
    )


def test_stationary_pose_has_zero_rate_and_expected_yaw():
    pose = rotation_z(math.pi / 4)
    np.testing.assert_allclose(
        world_angular_velocity(pose, pose, 0.02), [0, 0, 0], atol=1e-12
    )
    np.testing.assert_allclose(rpy_degrees(pose), [0, 0, 45], atol=1e-12)
