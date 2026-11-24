"""Regression fixtures for physical entry diagnostics, not a successful scoop."""

from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
from wheel_loader.environment.simulation.edge_entry import build_fixture, run_case

BUNDLE = Path("outputs/wa320/backend_benchmark/bundle")
pytestmark = pytest.mark.skipif(
    not (BUNDLE / "compiled_model.mjb").exists(),
    reason="Local preserved benchmark bundle required",
)


def test_fixture_preserves_compiled_colliders_and_materials():
    source = mujoco.MjModel.from_binary_path(str(BUNDLE / "compiled_model.mjb"))
    model, _ = build_fixture(BUNDLE, 3)
    assert model.neq == 0 and model.nu == 0 and model.nmocap == 0
    for g in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        old = mujoco.mj_name2id(
            source,
            mujoco.mjtObj.mjOBJ_GEOM,
            "grain_geom_0" if name.startswith("grain_geom_") else name,
        )
        for field in [
            "size",
            "pos",
            "quat",
            "friction",
            "solref",
            "solimp",
            "margin",
            "gap",
            "condim",
            "contype",
            "conaffinity",
        ]:
            np.testing.assert_array_equal(
                getattr(model, "geom_" + field)[g],
                getattr(source, "geom_" + field)[old],
            )
    old = mujoco.mj_name2id(source, mujoco.mjtObj.mjOBJ_BODY, "grain_0")
    for i in range(3):
        new = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"grain_{i}")
        assert model.body_mass[new] == source.body_mass[old]
        np.testing.assert_array_equal(model.body_inertia[new], source.body_inertia[old])
    np.testing.assert_array_equal(model.mesh_vert, source.mesh_vert)


def test_blunt_entry_push_is_physical_bounded_and_repeatable(tmp_path):
    a = run_case(BUNDLE, tmp_path / "a")
    run_case(BUNDLE, tmp_path / "b")
    one = np.load(tmp_path / "a/states.npz")
    two = np.load(tmp_path / "b/states.npz")
    np.testing.assert_array_equal(one["qpos"], two["qpos"])
    np.testing.assert_array_equal(one["contacts"], two["contacts"])
    assert (
        a["mouth_entry_events"] == 0 and a["particle_forward_displacement_m"][0] > 1.0
    )
    assert a["first_contact_normal_on_particle_world"][0] > 0.99
    assert a["sampled_forces_finite"] and not any(a["solver_warnings"])
    assert a["minimum_bucket_floor_clearance_m"] > 0.04
    assert a["maximum_accounting_error_kg"] < 1e-9
    assert abs(one["carriage_effort"]).max() <= 3e5
    assert (abs(one["tracking_error"]).max(0) < [0.01, 0.01, 0.01]).all()
    with pytest.raises(ValueError, match="empty"):
        run_case(BUNDLE, tmp_path / "a")


@pytest.mark.parametrize("count", [0, 2, 4])
def test_rejects_unsupported_fixture_sizes(count):
    with pytest.raises(ValueError):
        build_fixture(BUNDLE, count)
