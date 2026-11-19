"""Bucket registration, held-out pose and explicit rejection of an invalid rod."""

import copy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from wheel_loader.vehicle.brochure_geometry import rotation
from wheel_loader.vehicle.bucket_geometry import (
    fit_bucket_features,
    reconstruct_bucket,
    rod_lengths,
)


@pytest.fixture
def configs():
    root = Path(__file__).resolve().parents[2] / "configs/geometry"
    return tuple(
        yaml.safe_load((root / f).read_text())
        for f in ("wa320_7_brochure.yaml", "wa320_7_bucket.yaml")
    )


def test_known_rigid_bucket_features_recover_under_rotation_and_translation():
    expected = {
        "lip": np.array([1.4, -0.2]),
        "mouth_top": np.array([0.9, 1.1]),
        "anchor": np.array([0, 0.4]),
    }
    poses = {}
    for name, a, p in [
        ("ground", 0, np.array([1, 0.2])),
        ("rollback", 1.1, np.array([1.1, 4])),
    ]:
        poses[name] = {
            "P": p,
            "angle": a,
            **{k: p + rotation(a) @ v for k, v in expected.items()},
        }
    actual = fit_bucket_features(poses, ["ground", "rollback"])
    for k, value in expected.items():
        np.testing.assert_allclose(actual[k], value, atol=1e-12)
    with pytest.raises(ValueError):
        fit_bucket_features(poses, ["ground", "ground"])


def test_dump_is_held_out_and_cannot_change_fitted_bucket(configs):
    base, config = configs
    _, _, local, _, report = reconstruct_bucket(base, config)
    changed = copy.deepcopy(config)
    changed["poses"]["raised_dump"]["lip_px"][0] += 100
    _, _, other, _, result = reconstruct_bucket(base, changed)
    for key in local:
        np.testing.assert_array_equal(local[key], other[key])
    assert (
        result["pose_residuals_m"]["raised_dump"]["lip"]
        > report["pose_residuals_m"]["raised_dump"]["lip"] + 0.5
    )
    changed["fit_poses"] = ["ground", "raised_dump"]
    with pytest.raises(ValueError, match="Held-out"):
        reconstruct_bucket(base, changed)


def test_first_anchor_registers_better_but_rod_hypothesis_is_rejected(configs):
    _, _, _, _, report = reconstruct_bucket(*configs)
    errors = report["raised_anchor_ground_registration_errors_m"]
    assert errors["hypothesis_1"] < 0.05
    assert errors["hypothesis_2"] > 0.15
    assert not report["rod_hypothesis"]["fixed_D_single_rod_consistent"]
    assert report["rod_hypothesis"]["length_spread_m"] > 0.1
    assert (
        report["rod_hypothesis"]["constraint_rank"]
        < report["rod_hypothesis"]["unknown_D_coordinates"]
    )
    assert not report["rod_hypothesis"]["identified_D"]
    assert not any(
        report[k]
        for k in (
            "full_linkage_identified",
            "dynamics_validated",
            "production_geometry_promoted",
            "collision_ready",
            "capacity_reconstructed",
        )
    )


def test_rod_lengths_use_metres_and_actual_anchor_positions():
    model = SimpleNamespace(pose=lambda f: {"D": np.array([f, 0])})
    poses = {
        "a": {"fraction": 0, "anchor": np.array([0, 2])},
        "b": {"fraction": 1, "anchor": np.array([4, 4])},
    }
    np.testing.assert_allclose(rod_lengths(model, poses, ["a", "b"]), [2, 5])


def test_rigid_profile_and_features_preserve_distances_through_curl(configs):
    _, _, local, profile, _ = reconstruct_bucket(*configs)
    baseline = np.linalg.norm(profile[:, None] - profile[None, :], axis=-1)
    for a in np.linspace(-0.8, 1.2, 31):
        world = [2, 3] + profile @ rotation(a).T
        np.testing.assert_allclose(
            np.linalg.norm(world[:, None] - world[None, :], axis=-1),
            baseline,
            atol=1e-12,
        )
    np.testing.assert_array_equal(profile[0], local["lip"])
    np.testing.assert_array_equal(profile[-1], local["mouth_top"])


def test_clearance_and_reach_are_different_and_reach_uses_tire_envelope(configs):
    base, config = configs
    _, poses, local, _, report = reconstruct_bucket(base, config)
    p = poses["raised_dump"]
    lip = p["P"] + rotation(p["angle"]) @ local["lip"]
    cross_check = report["dump_geometry_cross_check"]
    assert cross_check["predicted_clearance_m"] == pytest.approx(lip[1])
    assert cross_check["predicted_reach_m"] == pytest.approx(lip[0] - 0.695)
    # Keep a visible reach mismatch rather than fitting the published dimension away.
    assert abs(cross_check["reach_error_m"]) > 0.08
