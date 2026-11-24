import numpy as np
import pytest

from wheel_loader.evaluation.backend_benchmark import (
    aligned,
    envelope_penetration,
    trajectory_error,
)


def test_envelope_uses_local_height_and_bucket_strip():
    centres = np.array([[5.0, 0, 1], [1.0, 4, 1]])
    lip = np.array([[4.0, 0, 1], [5.0, 0, 1], [6.0, 0, 4]])
    depth = envelope_penetration(lip, centres, np.ones(2), 2.0)
    np.testing.assert_allclose(depth[:2], [0.0, 1.0])
    assert np.isnan(depth[2])  # Above the pile is not deep penetration.


def test_envelope_sphere_cross_section_and_lateral_edge():
    depth = envelope_penetration([[5.0, 0, 1.6]], [[5.0, 1.5, 1]], [1.0], 2.0)
    np.testing.assert_allclose(depth, [np.sqrt(1 - 0.6**2 - 0.5**2)])


def test_refuse_time_shift_and_nonfinite_comparisons():
    a = {
        "time_s": np.array([0.0, 1]),
        "body_positions": np.zeros((2, 3)),
        "joint_pos": np.zeros((2, 1)),
    }
    with pytest.raises(ValueError, match="times differ"):
        aligned(a, {**a, "time_s": np.array([0.0, 2])})
    with pytest.raises(ValueError, match="Nonfinite"):
        aligned(a, {**a, "joint_pos": np.full((2, 1), np.nan)})


def test_trajectory_error_is_vector_distance_not_coordinate_average():
    error = trajectory_error(np.zeros((2, 3)), [[3.0, 4, 0], [0.0, 0, 0]])
    assert error["max_m"] == 5.0
    assert error["rms_m"] == pytest.approx(np.sqrt(12.5))
