"""Constraint-tangent construction and audit contracts, not machine validation."""

import importlib.util

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("mujoco") is None, reason="requires MuJoCo"
)


def test_tangent_satisfies_equality_and_requested_slider_speed():
    import mujoco

    from wheel_loader.environment.simulation.mechanism_investigation import (
        consistent_bucket_velocity,
        equality_jacobian,
    )

    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
    <body><joint name="bucket_slider_joint" type="slide"/><geom size=".1"/>
    <body><joint name="passive_joint" type="slide"/><geom size=".1"/></body></body>
    </worldbody><equality><joint joint1="bucket_slider_joint" joint2="passive_joint" polycoef="0 1 0 0 0"/></equality></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    tangent, meta = consistent_bucket_velocity(model, data, 1e-7)
    jac = equality_jacobian(model, data)
    np.testing.assert_allclose(jac @ tangent, 0, atol=1e-20)
    np.testing.assert_allclose(tangent, [1e-7, 1e-7], rtol=1e-12)
    assert abs((jac @ np.array([1e-7, 0]))[0]) > 0
    assert meta["max_equation_residual"] < 1e-20


def test_gpu_capacity_union_retains_a_cleared_transient_flag():
    wp = pytest.importorskip("warp")
    from wheel_loader.environment.simulation.gpu_capacity_audit import (
        accumulate_overflow,
    )

    wp.init()
    # CPU execution exercises the same Warp kernel without requiring a GPU in CI.
    current = wp.array([4], dtype=int, device="cpu")
    cumulative = wp.zeros(1, dtype=int, device="cpu")
    wp.launch(accumulate_overflow, dim=1, inputs=[current, cumulative], device="cpu")
    current.zero_()
    wp.launch(accumulate_overflow, dim=1, inputs=[current, cumulative], device="cpu")
    assert int(cumulative.numpy()[0]) == 4
