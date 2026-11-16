import numpy as np
import pytest
from wheel_loader_rl.material_metrics import settling_metrics, surface_slopes
from wheel_loader_rl.material_accounting import bucket_contents, bucket_local_positions, transfer_metrics
from wheel_loader_rl.controllers.scripted_loading import KEYFRAMES, loading_targets


def test_stationary_pile_and_continuing_drift():
    times = np.linspace(0, 3, 76)
    points = np.array([[0, 0, .2], [1, 0, .2], [0, 1, .4]])
    positions = np.broadcast_to(points, (76, 3, 3)).copy()
    assert settling_metrics(positions, times, .1)['settling_screen_passed']
    positions[:, :, 0] += times[:, None] * .1
    result = settling_metrics(positions, times, .1)
    assert not result['settling_screen_passed']
    assert result['speed_p95_mps'] == pytest.approx(.1)


def test_short_run_does_not_pass_settling():
    assert not settling_metrics(np.zeros((3, 2, 3)), [0, .1, .2], .1)['settling_screen_passed']


def test_known_roof_slopes():
    grid = np.arange(-3, 3.01, .05)
    x, y = np.meshgrid(grid, grid)
    z = np.maximum(0, 2 - np.maximum(abs(x), abs(y)) * np.tan(np.radians(30)))
    results = surface_slopes(np.column_stack((x.ravel(), y.ravel(), z.ravel())), .05)
    for fit in results.values():
        assert fit is not None
        assert fit['angle_deg'] == pytest.approx(30, abs=.5)
        assert fit['r_squared'] > .99


def test_bucket_transform_and_wall_exclusion():
    identity = [0, 0, 0, 1, 0, 0, 0]
    points = np.array([[.6, 0, .4], [.05, 0, .4], [1.6, 0, .4]])
    assert bucket_contents(points, identity, .1).tolist() == [True, False, False]
    # Rotate the bucket 90 degrees around world Z, then translate.
    q = np.sqrt(.5)
    pose = [2, 3, 1, q, 0, 0, q]
    assert bucket_local_positions([[2, 3.6, 1.4]], pose) == pytest.approx(np.array([[.6, 0, .4]]), abs=1e-12)
    with pytest.raises(ValueError):
        bucket_contents(points, [0, 0, 0, 0, 0, 0, 0], .1)


def test_transfer_tracks_same_ids_and_partitions_mass():
    times = [0, 16, 22, 30]
    positions = np.array([[[2, 0, .2], [3, 0, .2]], [[.6, 0, 1.4], [3, 0, .2]],
                          [[.6, 0, 1.4], [3, 0, .2]], [[.2, 0, .1], [3, 0, .2]]])
    poses = np.tile([0, 0, 1, 1, 0, 0, 0], (4, 1))
    result = transfer_metrics(positions, times, poses, .1, 2)
    assert result['acquired_particles'] == result['retained_particles'] == result['released_to_pad_particles'] == 1
    assert result['conservation_error_kg'] == 0
    assert result['transfer_screen_passed']
    assert not transfer_metrics(positions[:2], times[:2], poses[:2], .1, 2)['complete_sequence']


def test_script_respects_physical_rate_limits():
    rates = abs(np.diff(KEYFRAMES[:, 1:], axis=0) / np.diff(KEYFRAMES[:, 0])[:, None])
    assert np.all(rates <= np.array([1, .4, .6]) + 1e-9)
    assert loading_targets(200) == pytest.approx(KEYFRAMES[-1, 1:])
    with pytest.raises(ValueError):
        loading_targets(float('nan'))


def test_script_rejects_incompatible_configured_limits():
    from wheel_loader_rl.config import ActionLimits
    from wheel_loader_rl.controllers.scripted_loading import validate_loading_limits
    validate_loading_limits(ActionLimits())
    with pytest.raises(ValueError, match='configured'):
        validate_loading_limits(ActionLimits(longitudinal_mps=.2))
    assert np.all(KEYFRAMES[:, 1:] >= np.array([-1, 0, -1.3]))
    assert np.all(KEYFRAMES[:, 1:] <= np.array([4, .8, .9]))
