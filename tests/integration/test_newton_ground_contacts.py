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


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "requires MuJoCo")
class RigidPileContactsTests(unittest.TestCase):
    def test_bucket_pile_evidence_requires_geometry_and_separate_force(self):
        xml = """<mujoco><worldbody>
          <geom name="/World/Ground/Pile" type="box" size="1 1 1"/>
          <body name="bucket"><freejoint/><geom type="box" size=".2 .2 .2" mass="1"/></body>
        </worldbody></mujoco>"""
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "pile.xml"
            path.write_text(xml)
            roots = np.array([[1.19, 0, 0, 0, 0, 0, 1]])
            positions = np.empty((1, 0))
            measured = contact_sweep(
                path, positions, roots, [], np.array([[[3.0, 4.0, 0.0]]])
            )
            self.assertEqual(measured["bucket_pile_contact_samples"], 1)
            self.assertEqual(measured["peak_bucket_force_at_pile_contact_N"], 5)
            unforced = contact_sweep(path, positions, roots, [], np.zeros((1, 1, 3)))
            self.assertEqual(unforced["bucket_pile_contact_samples"], 1)
            self.assertEqual(unforced["peak_bucket_force_at_pile_contact_N"], 0)
            with self.assertRaises(ValueError):
                contact_sweep(path, positions, roots, [], np.zeros((2, 1, 3)))
            roots[0, 0] = 3
            clear = contact_sweep(
                path, positions, roots, [], np.array([[[3.0, 4.0, 0.0]]])
            )
            self.assertEqual(clear["bucket_pile_contact_samples"], 0)
            self.assertEqual(clear["peak_bucket_force_at_pile_contact_N"], 0)


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "requires MuJoCo")
class StaticContactAuditTests(unittest.TestCase):
    def test_floor_plus_pile_are_both_audited(self):
        from wheel_loader.environment.simulation.ground_contact import (
            GroundContactProfile,
            audit_ground_model,
        )

        wheels = "".join(
            f'<body pos="{i * 2} 0 1"><freejoint/>'
            f'<geom name="tire_contact_{i}" type="cylinder" size=".5 .2" solref=".04 1"/></body>'
            for i in range(4)
        )
        template = """<mujoco><option solver="Newton" cone="elliptic"/>
        <worldbody><geom type="plane" size="10 10 .1" solref=".04 1"/>
        <geom name="Pile" type="box" size="1 1 1" pos="20 0 1" solref="PILE_REF 1"/>
        WHEELS</worldbody></mujoco>""".replace("WHEELS", wheels)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "audit.xml"
            path.write_text(template.replace("PILE_REF", ".04"))
            self.assertTrue(
                all(audit_ground_model(path, GroundContactProfile()).values())
            )
            path.write_text(template.replace("PILE_REF", ".2"))
            self.assertFalse(
                audit_ground_model(path, GroundContactProfile())[
                    "contact_reference_consumed"
                ]
            )


if __name__ == "__main__":
    unittest.main()
