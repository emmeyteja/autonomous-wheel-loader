"""Native MuJoCo contact-law diagnostic; not the Newton loader terrain backend."""

import json
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict
from pathlib import Path

import numpy as np

from wheel_loader.evaluation.column import evaluate_column
from wheel_loader.terrain.column import Column


def run_column(config, material, output, *, seed=None, dt=None, radius=None):
    import mujoco

    settings = config["materials"][material]
    dt = config["dt_s"] if dt is None else dt
    duration = config["duration_s"]
    sample_hz = config["sample_hz"]
    if not (0 < dt <= 0.002 and duration >= 4 and 0 < sample_hz <= 1 / dt):
        raise ValueError("Invalid diagnostic timing")
    sample_steps = round(1 / (dt * sample_hz))
    if not np.isclose(sample_steps * dt * sample_hz, 1):
        raise ValueError("Sampling interval must be a whole number of solver steps")
    column = Column(
        config["column_radius_m"],
        config["column_height_m"],
        config["radius_m"] if radius is None else radius,
        config["seed"] if seed is None else seed,
        config["max_particles"],
        None
        if config.get("target_mass_kg") is None
        else config["target_mass_kg"] / settings["particle_density_kg_m3"],
    )
    xyz, radii = column.generate()
    root = ET.Element("mujoco", model="material_column_diagnostic")
    ET.SubElement(
        root,
        "option",
        timestep=str(dt),
        solver="CG",
        cone="elliptic",
        iterations="40",
        integrator="implicitfast",
    )
    ET.SubElement(root, "size", memory="256M")
    default = ET.SubElement(root, "default")
    ET.SubElement(
        default,
        "geom",
        density=str(settings["particle_density_kg_m3"]),
        friction=f"{settings['sliding_friction']} {settings['torsional_friction_m']} {settings['rolling_friction_m']}",
        condim="6",
        solref=".002 1",
        solimp=".95 .99 .001",
    )
    ET.SubElement(default, "joint", damping="0")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(
        world, "geom", name="floor", type="plane", size="2 2 .1", rgba=".23 .25 .27 1"
    )
    for i, (point, r) in enumerate(zip(xyz, radii)):
        body = ET.SubElement(
            world, "body", name=f"element_{i}", pos=" ".join(map(str, point))
        )
        ET.SubElement(body, "freejoint")
        ET.SubElement(body, "geom", type="sphere", size=str(r), rgba=".69 .56 .34 1")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "validation.json").unlink(missing_ok=True)
    mujoco.mj_saveLastXML(str(output / "effective_model.xml"), model)
    assert model.nbody == len(radii) + 1 and model.nq == len(radii) * 7
    assert np.all(model.geom_condim == 6)
    assert np.allclose(
        model.geom_friction,
        [
            settings["sliding_friction"],
            settings["torsional_friction_m"],
            settings["rolling_friction_m"],
        ],
    )
    assert np.all(model.dof_damping == 0)
    records, velocities, angular, times, energies = [], [], [], [], []
    min_separation = 0.0
    masses = model.body_mass[1:].copy()
    started = time.monotonic()

    def sample():
        qd = data.qvel.reshape(-1, 6)
        records.append(data.qpos.reshape(-1, 7)[:, :3].copy())
        velocities.append(qd[:, :3].copy())
        angular.append(qd[:, 3:].copy())
        times.append(float(data.time))
        energies.append(
            float(
                np.sum(
                    0.5 * masses * np.sum(qd[:, :3] ** 2, axis=1)
                    + 0.2 * masses * radii**2 * np.sum(qd[:, 3:] ** 2, axis=1)
                )
            )
        )

    sample()
    for step in range(round(duration / dt)):
        mujoco.mj_step(model, data)
        if data.ncon:
            min_separation = min(
                min_separation, float(data.contact.dist[: data.ncon].min())
            )
        if (step + 1) % sample_steps == 0:
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise RuntimeError("Nonfinite granular state")
            sample()
    records, velocities, angular = map(np.asarray, (records, velocities, angular))
    result = evaluate_column(
        times,
        records,
        velocities,
        radii,
        min_separation,
        int(data.warning.number.sum()),
    )
    rim_p95 = float(
        np.quantile(
            np.linalg.norm(angular[np.asarray(times) >= times[-1] - 2], axis=-1)
            * radii,
            0.95,
        )
    )
    result["checks"]["trailing_rotation"] = rim_p95 < 0.01
    result.update(
        {
            "material_candidate": material,
            "calibration_status": "not_validated",
            "runtime": f"native MuJoCo {mujoco.__version__}; CPU material diagnostic",
            "configuration": asdict(column),
            "contact_trial": settings,
            "preparation": {
                "protocol": "mass_normalized_expanded_lattice"
                if column.solid_volume_m3 is not None
                else "legacy_lattice",
                "release": "instantaneous_unconfined",
                "target_mass_kg": config.get("target_mass_kg"),
                "initial_height_m": float(
                    np.max(xyz[:, 2] + radii) - np.min(xyz[:, 2] - radii)
                ),
                "initial_outer_radius_m": float(
                    np.max(np.linalg.norm(xyz[:, :2], axis=1) + radii)
                ),
                "initial_bulk_density_kg_m3": float(
                    masses.sum() / (np.pi * column.radius**2 * column.height)
                ),
                "initial_speed_m_s": 0.0,
                "nominal_element_radius_m": column.element_radius,
                "actual_element_radius_range_m": [
                    float(radii.min()),
                    float(radii.max()),
                ],
                "settled_in_container": False,
            },
            "dt_s": dt,
            "duration_s": times[-1],
            "element_count": len(radii),
            "represented_mass_kg": float(masses.sum()),
            "mass_error_kg": float(
                abs(
                    masses.sum()
                    - np.sum(
                        4 / 3 * np.pi * radii**3 * settings["particle_density_kg_m3"]
                    )
                )
            ),
            "trailing_rim_speed_p95_m_s": rim_p95,
            "final_kinetic_energy_J": energies[-1],
            "arena_capacity_bytes": int(model.narena),
            "peak_arena_bytes": int(data.maxuse_arena),
            "wall_time_s": time.monotonic() - started,
            "natural_grain_distribution_reproduced": False,
            "newton_loader_coupling_validated": False,
        }
    )
    result["checks"]["represented_mass"] = result["mass_error_kg"] < 1e-8
    if column.solid_volume_m3 is not None:
        result["checks"]["target_mass"] = bool(
            abs(masses.sum() - config["target_mass_kg"]) < 1e-8
        )
        result["checks"]["initial_envelope"] = bool(
            np.isclose(
                result["preparation"]["initial_height_m"],
                column.height,
                atol=1e-10,
                rtol=0,
            )
            and np.isclose(
                result["preparation"]["initial_outer_radius_m"],
                column.radius,
                atol=1e-10,
                rtol=0,
            )
        )
    np.savez_compressed(
        output / "motion.npz",
        time_s=times,
        xyz=records.astype(np.float32),
        linear_velocity=velocities.astype(np.float32),
        angular_velocity=angular.astype(np.float32),
        radii=radii,
        masses_kg=masses,
        kinetic_energy_J=energies,
    )
    (output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    return result
