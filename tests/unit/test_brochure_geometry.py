"""Independent rigid reconstruction and explicit drawing-identifiability checks."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from wheel_loader.vehicle.brochure_geometry import (
    fit_rigid_motion,
    reconstruct,
    rotation,
)


@pytest.fixture
def config():
    return yaml.safe_load(
        (
            Path(__file__).resolve().parents[2]
            / "configs/geometry/wa320_7_brochure.yaml"
        ).read_text()
    )


def test_recovers_known_rotation_centre_from_multiple_pins():
    centre = np.array([1.2, -0.7])
    lower = np.array([[2.1, 0.3], [1.3, 2.5], [3, 1]])
    raised = centre + (lower - centre) @ rotation(0.8).T
    base, angle, r = fit_rigid_motion(lower, raised)
    np.testing.assert_allclose(base, centre, atol=1e-12)
    assert angle == pytest.approx(0.8)
    assert np.linalg.det(r) == pytest.approx(1)


@pytest.mark.parametrize(
    "lower,raised",
    [
        ([[0, 0]], [[1, 1]]),
        ([[0, 0], [0, 0]], [[1, 1], [1, 1]]),
        ([[0, 0], [1, 0]], [[2, 1], [3, 1]]),
        ([[0, 0], [np.nan, 0]], [[0, 0], [1, 0]]),
    ],
)
def test_rejects_unidentifiable_or_nonfinite_rotation_centre(lower, raised):
    with pytest.raises(ValueError):
        fit_rigid_motion(lower, raised)


def test_reconstructed_members_stay_rigid_and_return_to_start(config):
    model, _, report = reconstruct(config)
    distances = []
    for fraction in np.linspace(0, 1, 51):
        pose = model.pose(float(fraction))
        distances.append(
            [
                np.linalg.norm(pose[b] - pose[a])
                for a, b in [
                    ("B", "P"),
                    ("B", "C"),
                    ("C", "P"),
                    ("C", "E"),
                    ("C", "D"),
                    ("D", "E"),
                ]
            ]
        )
    np.testing.assert_allclose(np.ptp(distances, axis=0), 0, atol=1e-12)
    again, _, _ = reconstruct(config)
    np.testing.assert_array_equal(model.pose(0)["P"], again.pose(0)["P"])
    assert not report["full_linkage_identified"]
    assert not report["dynamics_validated"]
    assert not report["production_geometry_promoted"]
    with pytest.raises(ValueError):
        model.pose(1.01)


def test_drawing_dimensions_cross_check_without_fitting_them(config):
    _, _, report = reconstruct(config)
    assert abs(report["raised_pin_height_error_m"]) < 0.04
    assert report["independent_visible_base_error_m"] < 0.06
    assert abs(report["nominal_wheel_height_error_m"]) < 0.02
    # R3060 is the inside tire envelope, not a wheel-centre target.
    radii = report["turning_reference"]
    assert radii["outer_wheel_centre_radius_m"] == pytest.approx(5.38, abs=0.03)
    assert radii["inner_tire_envelope_radius_m"] == pytest.approx(3.06, abs=0.03)
    assert radii["derived_nominal_tire_width_m"] == pytest.approx(0.54)
    assert 0.5 < report["crank_pivot_in_boom_m"][1] < 0.7
    assert (
        report["crank_included_angle_deg"] < 90
    )  # Not the active straight-bar assumption.


def test_drawing_pixel_scale_is_explicit_and_rescalable(config):
    model, _, report = reconstruct(config)
    config["calibration"]["wheelbase_m"] *= 2
    scaled, _, other = reconstruct(config)
    np.testing.assert_allclose(scaled.base, 2 * model.base)
    assert other["boom_length_m"] == pytest.approx(2 * report["boom_length_m"])
