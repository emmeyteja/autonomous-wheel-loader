"""Optional native-runtime check of preparation metadata and report persistence."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


@unittest.skipUnless(
    importlib.util.find_spec("mujoco"), "Install the material-audit extra"
)
class MaterialColumnProbeTest(unittest.TestCase):
    def test_mass_control_survives_native_simulation_and_json_serialization(self):
        import numpy as np

        from wheel_loader.environment.simulation.column_probe import run_column

        config = {
            "column_radius_m": 0.04,
            "column_height_m": 0.06,
            "radius_m": 0.008,
            "seed": 42,
            "max_particles": 2500,
            "target_mass_kg": 0.04,
            "dt_s": 0.001,
            "duration_s": 4,
            "sample_hz": 25,
            "materials": {
                "gravel": {
                    "particle_density_kg_m3": 2590,
                    "sliding_friction": 0.65,
                    "rolling_friction_m": 0.003,
                    "torsional_friction_m": 0.001,
                }
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            result = run_column(config, "gravel", directory)
            persisted = json.loads((Path(directory) / "validation.json").read_text())
            self.assertEqual(result, persisted)
            self.assertTrue(persisted["checks"]["target_mass"])
            self.assertTrue(persisted["checks"]["initial_envelope"])
            self.assertTrue(persisted["checks"]["no_solver_warnings"])
            self.assertAlmostEqual(persisted["represented_mass_kg"], 0.04, places=10)
            self.assertFalse(persisted["reference_validation_passed"])
            with np.load(Path(directory) / "motion.npz") as recording:
                initial, radii = recording["xyz"][0], recording["radii"]
                self.assertAlmostEqual(
                    float(
                        np.max(initial[:, 2] + radii) - np.min(initial[:, 2] - radii)
                    ),
                    0.06,
                    places=7,
                )
                self.assertAlmostEqual(
                    float(np.max(np.linalg.norm(initial[:, :2], axis=1) + radii)),
                    0.04,
                    places=7,
                )
