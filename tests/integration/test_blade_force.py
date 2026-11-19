"""Native solver known-load check for tool force sign and reset behavior."""

import importlib.util
import unittest

import numpy as np


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "Install material-audit extra")
class BladeForceTests(unittest.TestCase):
    def test_known_weight_contact_reaction_and_reset(self):
        import mujoco

        from wheel_loader.environment.simulation.blade_probe import blade_wrench

        model = mujoco.MjModel.from_xml_string("""<mujoco><option timestep=".001"/>
        <worldbody><geom name="floor" type="plane" size="1 1 .1"/>
        <body pos="0 0 .05"><freejoint/><geom name="load" type="sphere" size=".05" mass="2"/></body>
        </worldbody></mujoco>""")
        data = mujoco.MjData(model)
        states = []
        for _ in range(2):
            mujoco.mj_resetData(model, data)
            for _ in range(1500):
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            load, moment = blade_wrench(model, data, model.geom("load").id)
            floor, _ = blade_wrench(model, data, model.geom("floor").id)
            np.testing.assert_allclose(load, [0, 0, 2 * 9.81], atol=1e-6)
            np.testing.assert_allclose(floor, -load, atol=1e-10)
            np.testing.assert_allclose(moment, 0, atol=1e-10)
            states.append(data.qpos.copy())
        np.testing.assert_array_equal(*states)

    def test_loaded_fixture_reset_restores_identical_driven_state(self):
        from pathlib import Path

        import mujoco
        import yaml

        from wheel_loader.environment.simulation.blade_probe import build_blade

        config = yaml.safe_load(
            (
                Path(__file__).resolve().parents[2]
                / "configs/experiments/blade_force.yaml"
            ).read_text()
        )
        model, _, _, initial = build_blade(config, 0.1)
        data = mujoco.MjData(model)
        states = []
        for _ in range(2):
            mujoco.mj_resetData(model, data)
            for step in range(200):
                data.mocap_pos[0] = np.asarray(initial) + [
                    step * config["dt_s"] * 0.01,
                    0,
                    0,
                ]
                mujoco.mj_step(model, data)
            states.append(np.concatenate([data.qpos, data.qvel]))
        np.testing.assert_array_equal(*states)
