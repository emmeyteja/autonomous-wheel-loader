"""Bounded adapter checks; synthetic fixtures test contracts, not loader accuracy."""

import hashlib
import importlib.util
import json
import runpy
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "requires MuJoCo")
class MechanicsBenchmarkTests(unittest.TestCase):
    def setUp(self):
        import mujoco

        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.parent = Path(self.temp.name) / "parent"
        self.parent.mkdir()
        xml = """<mujoco><option timestep="0.0005208333333333333"/>
        <worldbody><body name="test_rear_frame" pos="0 0 1.2"><freejoint name="root"/>
        <geom name="rear" size=".2" mass="10"/>
        <body name="test_front"><joint name="articulation_joint" axis="0 0 1"/>
        <geom name="front" size=".1" mass="2"/>
        <body name="test_boom" pos=".4 0 0"><joint name="boom_joint" axis="0 1 0"/>
        <geom name="arm" size=".1" mass="1"/>
        <body name="test_bucket" pos=".4 0 0"><joint name="bucket_joint" axis="0 1 0"/>
        <geom name="bucket" size=".1" mass="1"/></body></body></body></body>
        <body name="grain_0" pos="5 0 .8"><freejoint name="grain_free"/>
        <geom name="grain" size=".123456789" mass="3.123456789"/></body>
        </worldbody><actuator><motor joint="boom_joint"/></actuator></mujoco>"""
        (self.parent / "model.xml").write_text(xml)
        self.model = mujoco.MjModel.from_xml_path(str(self.parent / "model.xml"))
        mujoco.mj_saveModel(self.model, str(self.parent / "compiled_model.mjb"), None)
        q = self.model.qpos0.copy()
        q[7:10] = [0.1, 0.2, -0.3]  # Deliberately differs from model.qpos0.
        np.savez_compressed(
            self.parent / "command_tape.npz",
            initial_qpos=q,
            initial_qvel=np.zeros(self.model.nv),
            effort=np.ones((12, 1)),
            phase=np.full(12, "settle"),
            control_dt_s=1 / 240,
        )
        from wheel_loader.vehicle.parameters import load_parameters

        self.parameters = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
        manifest = {
            "schema_version": 1,
            "control_steps": 12,
            "control_dt_s": 1 / 240,
            "substeps": 8,
            "record_every_ticks": 1,
            "parameter_sha256": hashlib.sha256(
                json.dumps(self.parameters, sort_keys=True).encode()
            ).hexdigest(),
            "sha256": {
                n: hashlib.sha256((self.parent / n).read_bytes()).hexdigest()
                for n in ["compiled_model.mjb", "model.xml", "command_tape.npz"]
            },
        }
        (self.parent / "benchmark.json").write_text(json.dumps(manifest))
        self.prepare = runpy.run_path(
            str(ROOT / "scripts/prepare_mechanics_benchmark.py")
        )["prepare"]

    def test_removal_preserves_nonzero_release_and_commands(self):
        import mujoco

        from wheel_loader.environment.simulation.backend_benchmark import load_bundle

        output = Path(self.temp.name) / "ground"
        manifest = self.prepare(self.parent, output, "unloaded_ground", 0.05)
        model, tape, _ = load_bundle(output)
        self.assertEqual(manifest["diagnostic"]["particle_count"], 0)
        self.assertEqual(model.nbody, self.model.nbody - 1)
        np.testing.assert_array_equal(tape["initial_qpos"][7:10], [0.1, 0.2, -0.3])
        np.testing.assert_array_equal(tape["effort"], np.ones((12, 1)))
        self.assertEqual(manifest["diagnostic"]["initial_vehicle_position_error_m"], 0)
        self.assertTrue(any(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE))

    def test_fixed_empty_scene_runs_without_fake_material_events(self):
        import mujoco

        from wheel_loader.environment.simulation.backend_benchmark import (
            load_bundle,
            run_replay,
        )

        output = Path(self.temp.name) / "fixed"
        self.prepare(self.parent, output, "fixed_no_contact", 0.05)
        model, _, _ = load_bundle(output)
        self.assertFalse(any(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE))
        report = run_replay(output, output / "native", "native", self.parameters)
        self.assertTrue(report["completed"])
        self.assertFalse(report["material_present"])
        self.assertIsNone(report["acquisition_funnel"])
        self.assertEqual(report["sampling_hz"], 240)

    def test_solver_variant_keeps_exact_particle_geometry_and_mass(self):
        import mujoco

        from wheel_loader.environment.simulation.backend_benchmark import load_bundle

        output = Path(self.temp.name) / "pile"
        self.prepare(self.parent, output, "pile_reference", 0.05, solver="Newton")
        model, _, manifest = load_bundle(output)
        np.testing.assert_array_equal(model.body_mass, self.model.body_mass)
        np.testing.assert_array_equal(model.geom_size, self.model.geom_size)
        self.assertEqual(model.opt.solver, mujoco.mjtSolver.mjSOL_NEWTON)
        self.assertEqual(manifest["diagnostic"]["particle_count"], 1)

    def test_contact_ablation_preserves_floating_root(self):
        import mujoco

        from wheel_loader.environment.simulation.backend_benchmark import load_bundle

        output = Path(self.temp.name) / "floating_no_contact"
        self.prepare(self.parent, output, "floating_no_contact", 0.05)
        model, _, _ = load_bundle(output)
        self.assertTrue(any(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE))
        self.assertTrue(model.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_CONTACT)
