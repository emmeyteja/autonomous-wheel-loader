"""CPU collision evidence checks; optional MuJoCo dependency, no Isaac launch."""

import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np

from wheel_loader.environment.simulation.newton_ground import contact_sweep


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "requires MuJoCo")
class GroundContactsTests(unittest.TestCase):
    def test_contact_evidence_and_deep_penetration(self):
        wheels = "".join(
            f'<body name="{axle}_{side}_wheel" pos="{x} {y} 0">'
            '<geom type="sphere" size=".5" mass="1"/></body>'
            for axle, x in (("front", 1), ("rear", -1))
            for side, y in (("left", 1), ("right", -1))
        )
        xml = (
            '<mujoco><worldbody><geom type="plane" size="10 10 .1"/>'
            '<body name="rear_frame"><freejoint/><geom size=".1" mass="10"/>'
            f"{wheels}</body></worldbody></mujoco>"
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "contacts.xml"
            path.write_text(xml)
            roots = np.array([[0.0, 0, 0.499, 0, 0, 0, 1]])
            q = np.empty((1, 0))
            contact = contact_sweep(path, q, roots, [])
            self.assertTrue(contact["four_wheels_contact_ground"])
            self.assertEqual(contact["four_wheel_nonpositive_distance_fraction"], 1)
            self.assertTrue(contact["ground_penetration"])
            roots[0, 2] = 2
            clear = contact_sweep(path, q, roots, [])
            self.assertFalse(clear["four_wheels_contact_ground"])
            self.assertEqual(clear["four_wheel_nonpositive_distance_fraction"], 0)
            roots[0, 2] = 0.3
            deep = contact_sweep(path, q, roots, [])
            self.assertFalse(deep["ground_penetration"])


if __name__ == "__main__":
    unittest.main()
