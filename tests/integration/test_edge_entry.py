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


@pytest.mark.parametrize("variant,tip", [("taper_5mm", 0.005), ("taper_15mm", 0.015)])
def test_taper_changes_only_first_collider_and_preserves_bottom_and_inertia(
    variant, tip
):
    original, _ = build_fixture(BUNDLE, 1)
    model, _ = build_fixture(BUNDLE, 1, variant)
    changed = next(
        g
        for g in range(model.ngeom)
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g).endswith("box_12_4")
    )
    for field in [
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
            getattr(model, "geom_" + field), getattr(original, "geom_" + field)
        )
    for field in ["size", "pos", "quat", "type"]:
        a, b = getattr(model, "geom_" + field), getattr(original, "geom_" + field)
        np.testing.assert_array_equal(
            np.delete(a, changed, axis=0), np.delete(b, changed, axis=0)
        )
    for field in ["mass", "inertia", "ipos", "iquat"]:
        np.testing.assert_array_equal(
            getattr(model, "body_" + field), getattr(original, "body_" + field)
        )
    mesh = model.geom_dataid[changed]
    vertices = model.mesh_vert[
        model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]
    ]
    new_rotation, old_rotation = np.zeros(9), np.zeros(9)
    mujoco.mju_quat2Mat(new_rotation, model.geom_quat[changed])
    mujoco.mju_quat2Mat(old_rotation, original.geom_quat[changed])
    local = (
        vertices @ new_rotation.reshape(3, 3).T
        + model.geom_pos[changed]
        - original.geom_pos[changed]
    ) @ old_rotation.reshape(3, 3)
    hx, hy, hz = original.geom_size[changed]
    np.testing.assert_allclose(local.min(0), [-hx, -hy, -hz], atol=2e-6)
    np.testing.assert_allclose(local.max(0), [hx, hy, hz], atol=2e-6)
    front = local[abs(local[:, 0] + hx) < 2e-6]
    np.testing.assert_allclose(front[:, 2].max() - front[:, 2].min(), tip, atol=2e-6)
    assert np.any(
        (abs(local[:, 0] - (-hx + 0.15)) < 2e-6) & (abs(local[:, 2] + hz) < 2e-6)
    )
    with pytest.raises(ValueError, match="variant"):
        build_fixture(BUNDLE, 1, "unknown")


def test_backstop_fixture_keeps_bucket_clear_of_wall_and_reports_stall(tmp_path):
    r = run_case(BUNDLE, tmp_path / "stop", variant="taper_5mm", backstop=True)
    s = np.load(tmp_path / "stop/states.npz")
    assert r["backstop"] and r["minimum_bucket_backstop_gap_m"] > 0.15
    assert r["maximum_carriage_advance_m"] == 0.70
    assert r["sampled_forces_finite"] and not any(r["solver_warnings"])
    assert r["maximum_accounting_error_kg"] < 1e-9
    assert r["mouth_entry_events"] == 0
    assert abs(s["carriage_effort"]).max() <= 3e5
    assert max(r["maximum_carriage_tracking_error_m_m_rad"]) > 0.01
