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
            from wheel_loader.vehicle.parameters import load_parameters

            self.assertEqual(
                len(bodies),
                len(load_parameters(ROOT / "configs/vehicle/wa320.yaml")["bodies"]),
            )
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


class NewtonProfileAssetTests(unittest.TestCase):
    def test_numerical_profile_preserves_vehicle_and_passive_drives(self):
        import hashlib
        import tempfile

        from wheel_loader.environment.simulation.newton_profile import (
            BOUNDED_JOINTS,
            NewtonProfile,
            prepare_asset,
        )

        source = ROOT / "assets/wheel_loader/wa320/usd/wa320_fixed.usda"
        original_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        settings = NewtonProfile()
        with tempfile.TemporaryDirectory() as temporary:
            output = prepare_asset(
                source,
                Path(temporary) / "prepared.usda",
                settings,
                require_fixed_base=True,
            )
            stage = Usd.Stage.Open(str(output))
            original = Usd.Stage.Open(str(source))
            for prim in stage.Traverse():
                before = original.GetPrimAtPath(prim.GetPath())
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    for name in (
                        "physics:mass",
                        "physics:centerOfMass",
                        "physics:diagonalInertia",
                        "physics:principalAxes",
                    ):
                        self.assertEqual(
                            prim.GetAttribute(name).Get(),
                            before.GetAttribute(name).Get(),
                        )
                if prim.GetName() in BOUNDED_JOINTS:
                    self.assertAlmostEqual(
                        prim.GetAttribute("mjc:solreflimit").Get()[0],
                        settings.stop_time_constant,
                    )
                    self.assertAlmostEqual(
                        prim.GetAttribute("mjc:solimplimit").Get()[0],
                        settings.stop_impedance,
                    )
                if prim.GetName() in ("rod_joint", "bucket_joint"):
                    self.assertFalse(prim.HasAPI(UsdPhysics.DriveAPI, "angular"))
            self.assertFalse(stage.GetPrimAtPath("/wa320/zbar_closure").IsActive())
            self.assertTrue(original.GetPrimAtPath("/wa320/zbar_closure").IsActive())
            constraints = [
                prim
                for prim in stage.Traverse()
                if "MjcEqualityConnectAPI"
                in (
                    prim.GetMetadata("apiSchemas").GetAddedOrExplicitItems()
                    if prim.GetMetadata("apiSchemas")
                    else []
                )
            ]
            self.assertEqual(len(constraints), 2)
            for index, prim in enumerate(
                sorted(constraints, key=lambda p: str(p.GetPath()))
            ):
                joint = UsdPhysics.SphericalJoint(prim)
                self.assertTrue(joint.GetExcludeFromArticulationAttr().Get())
                self.assertEqual(joint.GetBody0Rel().GetTargets()[0].name, "rod")
                self.assertEqual(joint.GetBody1Rel().GetTargets()[0].name, "bucket")
                self.assertAlmostEqual(joint.GetLocalPos0Attr().Get()[1], index * 0.1)
                self.assertAlmostEqual(joint.GetLocalPos1Attr().Get()[1], index * 0.1)
                self.assertAlmostEqual(
                    prim.GetAttribute("mjc:solref").Get()[0],
                    settings.loop_time_constant,
                )
            self.assertEqual(
                hashlib.sha256(source.read_bytes()).hexdigest(), original_hash
            )
            with self.assertRaises(ValueError):
                prepare_asset(source, source, settings)
            with self.assertRaises(ValueError):
                prepare_asset(
                    ROOT / "assets/wheel_loader/wa320/usd/wa320.usda",
                    Path(temporary) / "bad.usda",
                    settings,
                    require_fixed_base=True,
                )


class NewtonTireContactTests(unittest.TestCase):
    def test_derived_cylinders_preserve_mass_dimensions_and_source(self):
        import hashlib
        import tempfile

        from pxr import UsdGeom

        from wheel_loader.environment.simulation.ground_contact import (
            GroundContactProfile,
            author_tire_contacts,
        )
        from wheel_loader.environment.simulation.newton_profile import (
            NewtonProfile,
            prepare_asset,
        )
        from wheel_loader.vehicle.parameters import load_parameters

        source = ROOT / "assets/wheel_loader/wa320/usd/wa320.usda"
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
        with tempfile.TemporaryDirectory() as temporary:
            derived = prepare_asset(
                source, Path(temporary) / "newton.usda", NewtonProfile()
            )
            stage = Usd.Stage.Open(str(derived))
            before = {
                str(prim.GetPath()): tuple(
                    prim.GetAttribute(name).Get()
                    for name in (
                        "physics:mass",
                        "physics:centerOfMass",
                        "physics:diagonalInertia",
                        "physics:principalAxes",
                    )
                )
                for prim in stage.Traverse()
                if prim.HasAPI(UsdPhysics.RigidBodyAPI)
            }
            settings = GroundContactProfile()
            author_tire_contacts(stage, p, settings)
            tires = [
                prim for prim in stage.Traverse() if prim.GetName() == "tire_contact"
            ]
            self.assertEqual(len(tires), 4)
            for prim in tires:
                self.assertTrue(prim.IsA(UsdGeom.Cylinder))
                shape = UsdGeom.Cylinder(prim)
                self.assertEqual(shape.GetAxisAttr().Get(), "Y")
                self.assertAlmostEqual(
                    shape.GetRadiusAttr().Get(), p["vehicle"]["wheel_radius"]
                )
                self.assertAlmostEqual(
                    shape.GetHeightAttr().Get(), p["vehicle"]["wheel_width"]
                )
                self.assertTrue(prim.HasAPI(UsdPhysics.CollisionAPI))
                self.assertFalse(prim.HasAPI(UsdPhysics.MeshCollisionAPI))
                self.assertAlmostEqual(
                    prim.GetAttribute("mjc:solref").Get()[0],
                    settings.contact_time_constant,
                )
            for path, values in before.items():
                prim = stage.GetPrimAtPath(path)
                self.assertEqual(
                    values,
                    tuple(
                        prim.GetAttribute(name).Get()
                        for name in (
                            "physics:mass",
                            "physics:centerOfMass",
                            "physics:diagonalInertia",
                            "physics:principalAxes",
                        )
                    ),
                )
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), digest)


if __name__ == "__main__":
    unittest.main()
