"""Isolated native CPU blade/material diagnostic, outside the vehicle backend."""

import csv
import hashlib
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from wheel_loader.evaluation.blade import contact_wrench_on_geom, fit_draft_force
from wheel_loader.terrain.bed import RectangularBed


def build_blade(config, width, *, empty=False):
    import mujoco

    material = config["materials"][config["material"]]
    bed = RectangularBed(
        tuple(config["bed_size_m"]),
        config["radius_m"],
        config["initial_mass_kg"] / material["density_kg_m3"],
        config["seed"],
        config["max_particles"],
    )
    points, radii = bed.generate()
    if empty:
        points, radii = points[:0], radii[:0]
    root = ET.Element("mujoco", model="unmatched_blade_diagnostic")
    ET.SubElement(
        root,
        "option",
        timestep=str(config["dt_s"]),
        integrator="implicitfast",
        solver="CG",
        iterations="80",
        cone="elliptic",
    )
    ET.SubElement(root, "size", memory="256M")
    default = ET.SubElement(root, "default")
    ET.SubElement(
        default,
        "geom",
        condim="6",
        solref=".002 1",
        solimp=".95 .99 .001",
        friction=" ".join(map(str, material["grain_friction"])),
        contype="1",
        conaffinity="7",
    )
    world = ET.SubElement(root, "worldbody")
    wall = {
        "type": "box",
        "priority": "1",
        "contype": "2",
        "conaffinity": "1",
        "friction": " ".join(map(str, material["wall_friction"])),
        "rgba": ".35 .40 .45 .18",
    }
    ET.SubElement(
        world,
        "geom",
        name="floor",
        type="plane",
        size="1 1 .01",
        priority="1",
        contype="2",
        conaffinity="1",
        friction=wall["friction"],
        rgba=".24 .27 .29 1",
    )
    for name, pos, size in [
        ("back", "-.005 0 .09", ".005 .16 .09"),
        ("front", ".305 0 .09", ".005 .16 .09"),
        ("left", ".15 .155 .09", ".15 .005 .09"),
        ("right", ".15 -.155 .09", ".15 .005 .09"),
    ]:
        ET.SubElement(world, "geom", name=name, pos=pos, size=size, **wall)
    initial = [
        config["blade_start_x_m"],
        0,
        config["bed_size_m"][2] + config["blade_height_m"] / 2 + 0.06,
    ]
    position = " ".join(map(str, initial))
    ET.SubElement(world, "body", name="target", mocap="true", pos=position)
    blade = ET.SubElement(world, "body", name="blade", pos=position)
    ET.SubElement(blade, "freejoint")
    ET.SubElement(
        blade,
        "geom",
        name="blade_geom",
        type="box",
        size=f"{config['blade_thickness_m'] / 2} {width / 2} {config['blade_height_m'] / 2}",
        mass=str(config["blade_mass_kg"]),
        priority="1",
        contype="4",
        conaffinity="1",
        friction=" ".join(map(str, material["blade_friction"])),
        rgba=".25 .6 .85 1",
    )
    equality = ET.SubElement(root, "equality")
    # Numerical orientation fixture: 1 m torque weighting is not blade geometry.
    ET.SubElement(
        equality,
        "weld",
        body1="target",
        body2="blade",
        solref=".002 1",
        solimp=".95 .99 .001",
        torquescale="1",
    )
    for i, (point, r) in enumerate(zip(points, radii)):
        body = ET.SubElement(
            world, "body", name=f"grain_{i}", pos=" ".join(map(str, point))
        )
        ET.SubElement(body, "freejoint")
        ET.SubElement(
            body,
            "geom",
            name=f"grain_geom_{i}",
            type="sphere",
            size=str(r),
            density=str(material["density_kg_m3"]),
            rgba=".70 .56 .32 1",
        )
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return model, radii, points, initial


def blade_wrench(model, data, geom_id):
    import mujoco

    force, moment = np.zeros(3), np.zeros(3)
    body = model.geom_bodyid[geom_id]
    local = np.empty(6)
    contacts = data.contact
    indices = np.flatnonzero(
        (contacts.geom1[: data.ncon] == geom_id)
        | (contacts.geom2[: data.ncon] == geom_id)
    )
    for i in indices:
        c = data.contact[i]
        mujoco.mj_contactForce(model, data, i, local)
        f, m = contact_wrench_on_geom(
            c.geom1, c.geom2, c.frame, local, c.pos, data.xpos[body], geom_id
        )
        force += f
        moment += m
    return force, moment


def run_blade(config, width, depth, output, *, empty=False):
    import mujoco

    dt = float(config["dt_s"])
    hz = float(config["sample_hz"])
    if (
        not 0 < dt <= 0.002
        or not 0 < hz <= 1 / dt
        or not np.isclose(round(1 / (dt * hz)) * dt * hz, 1)
    ):
        raise ValueError("Invalid integration/sampling interval")
    if not (
        0 < width < 0.28
        and depth > 0
        and config["speed_m_s"] > 0
        and config["travel_m"] > 0
    ):
        raise ValueError("Invalid blade width/depth/travel")
    settle, insert, hold, relax = [
        float(config[k]) for k in ("settle_s", "insert_s", "hold_s", "relax_s")
    ]
    if min(settle, insert, hold, relax) <= 0 or settle < 1 or hold < 1:
        raise ValueError("Need positive phases and settling/hold windows")
    start_push = settle + insert + hold
    stop_push = start_push + config["travel_m"] / config["speed_m_s"]
    duration = stop_push + relax
    if not np.isclose(round(duration / dt) * dt, duration):
        raise ValueError("Duration must fit solver step")
    if (
        config["blade_start_x_m"] - config["blade_thickness_m"] / 2 <= 0
        or config["blade_start_x_m"]
        + config["travel_m"]
        + config["blade_thickness_m"] / 2
        >= 0.3
    ):
        raise ValueError("Blade travel would intersect the bin end walls")
    model, radii, points, initial = build_blade(config, width, empty=empty)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    ids = np.array([model.body(f"grain_{i}").id for i in range(len(radii))], dtype=int)
    blade_id = model.body("blade").id
    geom = model.geom("blade_geom").id
    masses = model.body_mass[ids]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "validation.json").unlink(missing_ok=True)
    mujoco.mj_saveLastXML(str(output / "effective_model.xml"), model)
    sample_steps = round(1 / (dt * hz))
    frames = []
    poses = []
    quats = []
    frame_times = []
    frame_phases = []
    times = []
    forces = []
    moments = []
    positions = []
    velocities = []
    targets = []
    phases = []
    settle_speeds = []
    hold_speeds = []
    max_tracking = 0.0
    max_angle_error = 0.0
    min_sep = 0.0
    escaped = False
    surface = None
    target_z = initial[2]
    started = time.monotonic()
    for step in range(round(duration / dt) + 1):
        t = step * dt
        if surface is None and t >= settle:
            mujoco.mj_forward(model, data)
            surface = (
                float(np.quantile(data.xpos[ids, 2] + radii, 0.95))
                if len(ids)
                else config["bed_size_m"][2]
            )
            target_z = surface - depth + config["blade_height_m"] / 2
            if surface - depth < 0.015:
                raise ValueError("Cut would approach floor; use a deeper prepared bed")
        target = np.array(initial, dtype=float)
        if t < settle:
            phase = 0
        elif t < settle + insert:
            phase = 1
            u = (t - settle) / insert
            target[2] = initial[2] + (target_z - initial[2]) * u * u * (3 - 2 * u)
        elif t < start_push:
            phase = 2
            target[2] = target_z
        elif t < stop_push:
            phase = 3
            target[2] = target_z
            target[0] += config["speed_m_s"] * (t - start_push)
        else:
            phase = 4
            target[2] = target_z
            target[0] += config["travel_m"]
        data.mocap_pos[0] = target
        # Split stepping keeps positions/velocities and contact forces at the same
        # pre-integration timestamp. The regular welded blade has real velocity;
        # a directly teleported mocap collider would omit its contact velocity.
        mujoco.mj_step1(model, data)
        actual = data.xpos[blade_id].copy()
        velocity = data.cvel[blade_id, 3:].copy()
        if len(ids):
            grain_velocity = data.cvel[ids, 3:]
            speed = float(np.quantile(np.linalg.norm(grain_velocity, axis=1), 0.95))
            if settle - 0.5 <= t < settle:
                settle_speeds.append(speed)
            if start_push - 0.5 <= t < start_push:
                hold_speeds.append(speed)
            xyz = data.xpos[ids]
            escaped = escaped or bool(
                np.any(
                    (xyz[:, 0] < -0.01)
                    | (xyz[:, 0] > 0.31)
                    | (np.abs(xyz[:, 1]) > 0.16)
                    | (xyz[:, 2] < 0)
                )
            )
        mujoco.mj_step2(model, data)
        f, m = blade_wrench(model, data, geom)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError("Nonfinite blade/material state")
        if data.ncon:
            min_sep = min(min_sep, float(np.min(data.contact.dist[: data.ncon])))
        max_tracking = max(max_tracking, float(np.linalg.norm(actual - target)))
        max_angle_error = max(
            max_angle_error,
            float(2 * np.arccos(np.clip(abs(data.xquat[blade_id, 0]), 0, 1))),
        )
        times.append(t)
        forces.append(f)
        moments.append(m)
        positions.append(actual)
        velocities.append(velocity)
        targets.append(target)
        phases.append(phase)
        if step % sample_steps == 0:
            frame_times.append(t)
            frames.append(data.xpos[ids].copy())
            poses.append(actual)
            quats.append(data.xquat[blade_id].copy())
            frame_phases.append(phase)
    times, forces, moments, positions, velocities, targets, phases = map(
        np.asarray, (times, forces, moments, positions, velocities, targets, phases)
    )
    cutting = phases == 3
    travel = positions[cutting, 0] - config["blade_start_x_m"]
    fit = fit_draft_force(travel, forces[cutting, 0], config["fit_window_m"])
    push_speed_error = float(
        np.quantile(
            np.abs(
                velocities[cutting & (times > start_push + 0.25), 0]
                - config["speed_m_s"]
            ),
            0.99,
        )
    )
    mass_error = float(abs(masses.sum() - (0 if empty else config["initial_mass_kg"])))
    draft = -forces[cutting, 0]
    work = float(
        np.sum((draft[1:] + draft[:-1]) * 0.5 * np.diff(positions[cutting, 0]))
    )
    checks = {
        "finite_states_and_wrenches": bool(
            np.isfinite(forces).all() and np.isfinite(moments).all()
        ),
        "no_solver_warnings": int(data.warning.number.sum()) == 0,
        "limited_overlap": min_sep
        >= (-0.15 * float(radii.min()) if len(radii) else -0.001),
        "tracking_under_2mm": max_tracking < 0.002,
        "blade_angle_error_under_0_01_rad": max_angle_error < 0.01,
        "cut_speed_error_under_1mm_s": push_speed_error < 0.001,
        "represented_mass": mass_error < 1e-8,
        "all_grains_retained_in_bin": not escaped,
        "bed_settled_before_insertion": empty
        or bool(settle_speeds and np.quantile(settle_speeds, 0.95) < 0.005),
        "bed_settled_before_cutting": empty
        or bool(hold_speeds and np.quantile(hold_speeds, 0.95) < 0.005),
        "measurable_opposing_draft": bool(np.max(np.abs(forces)) < 1e-8)
        if empty
        else bool(-np.mean(forces[cutting, 0]) > 0.001 and work > 0),
    }
    result = {
        "checks": checks,
        "runtime": f"native CPU MuJoCo {mujoco.__version__}",
        "material": config["material"],
        "empty_control": empty,
        "width_m": width,
        "depth_m": depth,
        "config": config,
        "element_count": len(radii),
        "represented_mass_kg": float(masses.sum()),
        "initial_grain_state_sha256": hashlib.sha256(
            points.tobytes() + radii.tobytes()
        ).hexdigest(),
        "surface_H95_before_insertion_m": surface,
        "initial_mass_error_kg": mass_error,
        "minimum_separation_m": min_sep,
        "max_blade_tracking_error_m": max_tracking,
        "max_blade_angle_error_rad": max_angle_error,
        "cut_speed_error_p99_m_s": push_speed_error,
        "cut_work_J": work,
        "max_abs_contact_force_N": float(np.max(np.abs(forces))),
        "force_fit": fit,
        "duration_s": duration,
        "wall_time_s": time.monotonic() - started,
        "peak_arena_bytes": int(data.maxuse_arena),
        "physical_validation_passed": False,
        "reference_status": "Unmatched diagnostic: coarse freely rotating spheres, trial material, fill and insertion; no measured force trace",
        "production_granular_backend_enabled": False,
    }
    np.savez_compressed(
        output / "motion.npz",
        time_s=frame_times,
        xyz=np.asarray(frames, dtype=np.float32),
        blade_xyz=poses,
        blade_quat_wxyz=quats,
        phase=frame_phases,
        radii=radii,
        force_time_s=times,
        force_world_N=forces,
        moment_world_Nm=moments,
        blade_position_m=positions,
        blade_velocity_m_s=velocities,
        target_position_m=targets,
        force_phase=phases,
    )
    with (output / "force.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "phase",
                "blade_x_m",
                "blade_z_m",
                "vx_m_s",
                "reaction_Fx_N",
                "reaction_Fy_N",
                "reaction_Fz_N",
                "Mx_Nm",
                "My_Nm",
                "Mz_Nm",
            ]
        )
        for i, t in enumerate(times):
            writer.writerow(
                [
                    t,
                    int(phases[i]),
                    positions[i, 0],
                    positions[i, 2],
                    velocities[i, 0],
                    *forces[i],
                    *moments[i],
                ]
            )
    (output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    return result
