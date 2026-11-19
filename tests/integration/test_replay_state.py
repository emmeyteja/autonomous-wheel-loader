"""Check world telemetry against an analytically known rigid-body motion."""

import importlib.util
import unittest
from pathlib import Path

import numpy as np

from wheel_loader.vehicle.parameters import load_parameters


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "requires MuJoCo")
class ReplayWorldStateTests(unittest.TestCase):
    def test_rotating_cab_offset_and_visual_pin_rates(self):
        import mujoco

        from wheel_loader.environment.simulation.replay_state import ReplayState

        xml = """<mujoco><worldbody><body name="test_rear_frame"><freejoint/>
          <geom size=".1" mass="10"/>
          <body name="test_front_frame"><joint name="articulation_joint"/>
            <geom size=".1" mass="1"/>
            <body name="test_boom"><joint name="boom_joint" axis="0 -1 0"/>
              <geom size=".1" mass="1"/>
              <body name="test_bucket"><joint name="bucket_joint" axis="0 -1 0"/>
                <geom size=".1" mass="1"/></body>
              <body name="test_bellcrank"><joint name="tilt_joint" axis="0 -1 0"/>
                <geom size=".1" mass="1"/>
                <body name="test_rod"><geom size=".1" mass="1"/></body>
              </body></body></body></body></worldbody></mujoco>"""
        model = mujoco.MjModel.from_xml_string(xml)
        parameters = load_parameters(
            Path(__file__).resolve().parents[2] / "configs/vehicle/wa320.yaml"
        )
        parameters["geometry"]["cab_center"] = [0, 2, 0]
        times = np.array([0.0, 0.001, 0.002])
        yaw = 0.2 * times
        roots = np.column_stack(
            (
                0.1 * times,
                times * 0,
                times * 0 + 1,
                times * 0,
                times * 0,
                np.sin(yaw / 2),
                np.cos(yaw / 2),
            )
        )
        names = ["articulation_joint", "boom_joint", "bucket_joint", "tilt_joint"]
        state = ReplayState(model, names, np.zeros((3, 4)), roots, times, parameters)
        sample = state.sample(1)
        velocity, omega = sample["velocities"]["CABIN (cab centre)"]
        expected = [0.1 - 0.4 * np.cos(yaw[1]), -0.4 * np.sin(yaw[1]), 0]
        np.testing.assert_allclose(velocity, expected, atol=1e-8)
        np.testing.assert_allclose(omega, [0, 0, 0.2], atol=1e-8)
        np.testing.assert_allclose(
            sample["velocities"]["HEAD (front hinge)"][0], [0.1, 0, 0], atol=1e-8
        )
        # Rigid world translation/rotation cannot change cylinder pin distances.
        np.testing.assert_allclose(list(sample["length_rates"].values()), 0, atol=1e-9)
        np.testing.assert_allclose(
            list(sample["length_changes"].values()), 0, atol=1e-9
        )
        np.testing.assert_allclose(
            state.data.qpos[state.base : state.base + 3], roots[1, :3]
        )
