import numpy as np

from wheel_loader.control.trajectory_feedback import BoundedTrajectoryFeedback


def test_common_feedback_uses_feedforward_error_and_shared_limits():
    controller = BoundedTrajectoryFeedback(
        [[1, 2]], [[0, 0]], [[3, 4]], [2, 2], [1, 1], [5, 5]
    )
    np.testing.assert_array_equal(
        controller.effort(0, np.array([0, 1]), np.array([1, 0])), [4, 5]
    )
    np.testing.assert_array_equal(
        controller.effort(0, np.array([1, 2]), np.array([0, 0])), [3, 4]
    )
