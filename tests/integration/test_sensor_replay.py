"""Native raycast physics geometry controls; no ROS/Isaac dependency."""

import importlib.util
import unittest

import numpy as np


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "Install material-audit extra")
class RaycastTests(unittest.TestCase):
    def test_known_box_plane_occlusion_miss_and_reset(self):
        import mujoco

        from wheel_loader.environment.simulation.sensor_replay import (
            raycast,
            sample_indices,
        )

        model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody><geom type="plane" size="10 10 .1"/>
        <geom type="box" pos="3 0 1" size=".5 .5 .5"/>
        <body pos="1 0 1"><freejoint/><geom type="box" size=".1 .1 .1"/></body>
        </worldbody></mujoco>""")
        data = mujoco.MjData(model)
        records = []
        for _ in range(2):
            mujoco.mj_resetData(model, data)
            mujoco.mj_forward(model, data)
            ranges, _ = raycast(
                model,
                data,
                [0, 0, 1],
                np.array([[1.0, 0, 0], [0, 0, -1], [-1, 0, 0]]),
                10,
            )
            np.testing.assert_allclose(ranges[:2], [0.9, 1.0], atol=1e-12)
            self.assertTrue(np.isnan(ranges[2]))
            records.append(ranges)
        np.testing.assert_allclose(*records, equal_nan=True)
        self.assertEqual(len(sample_indices(np.arange(241) / 240, 60)), 61)
        with self.assertRaises(ValueError):
            sample_indices(np.arange(241) / 240, 100)
