"""Run in the standalone USD importer environment: python -m unittest discover -s tests/integration.

No Kit launch or GPU needed. These checks validate authoring, not machine physics.
"""

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[2]


class ConvertedAssetTests(unittest.TestCase):
    def test_portability_and_physics_contract(self):
        for filename, fixed in [("wa320.usda", False), ("wa320_fixed.usda", True)]:
            path = ROOT / "assets/wheel_loader/wa320/usd" / filename
            text = path.read_text()
            for private in ("/home/", "/tmp/", "@http", "@file:"):
                self.assertNotIn(private, text)
            stage = Usd.Stage.Open(str(path))
            self.assertEqual(stage.GetRootLayer().subLayerPaths, [])
            self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
            bodies = {
                prim.GetName(): prim
                for prim in stage.Traverse()
                if prim.HasAPI(UsdPhysics.RigidBodyAPI)
            }
            self.assertEqual(len(bodies), 10)
            mass = sum(
                UsdPhysics.MassAPI(prim).GetMassAttr().Get() for prim in bodies.values()
            )
            self.assertAlmostEqual(mass, 15175.0)
            loops = [
                prim for prim in stage.Traverse() if prim.GetName() == "zbar_closure"
            ]
            self.assertEqual(len(loops), 1)
            loop = UsdPhysics.RevoluteJoint(loops[0])
            self.assertTrue(loop.GetExcludeFromArticulationAttr().Get())
            self.assertEqual(loop.GetBody0Rel().GetTargets(), [bodies["rod"].GetPath()])
            self.assertEqual(
                loop.GetBody1Rel().GetTargets(), [bodies["bucket"].GetPath()]
            )
            for prim in stage.Traverse():
                if prim.GetName() in ("bucket_joint", "rod_joint"):
                    self.assertFalse(prim.HasAPI(UsdPhysics.DriveAPI, "angular"))
                if prim.GetName().endswith("wheel_joint"):
                    self.assertEqual(
                        UsdPhysics.DriveAPI(prim, "angular").GetStiffnessAttr().Get(),
                        0.0,
                    )
            tires = [
                prim for prim in stage.Traverse() if prim.GetName() == "tire_contact"
            ]
            self.assertEqual(len(tires), 4)
            for tire in tires:
                self.assertEqual(len(UsdGeom.Mesh(tire).GetPointsAttr().Get()), 128)
                self.assertTrue(
                    UsdPhysics.CollisionAPI(tire).GetCollisionEnabledAttr().Get()
                )
            anchors = [
                prim for prim in stage.Traverse() if prim.GetName() == "root_joint"
            ]
            self.assertEqual(len(anchors), int(fixed))
            # Every visible/link body comes from the independently generated tree.
            tree = ET.parse(ROOT / "assets/wheel_loader/wa320/wa320.urdf")
            self.assertEqual(
                set(bodies), {link.attrib["name"] for link in tree.findall("link")}
            )


if __name__ == "__main__":
    unittest.main()
