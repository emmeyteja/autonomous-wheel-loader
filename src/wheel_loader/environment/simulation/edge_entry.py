"""Isolated finite-force carriage contact diagnostic; never a loader controller."""

import copy
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from wheel_loader.evaluation.material_accounting import (
    AcquisitionFunnel,
    MaterialAccounting,
    bucket_payload_mask,
)
from wheel_loader.vehicle.parameters import load_parameters


def build_fixture(bundle, count):
    """Reuse bucket, floor and first particle; omit the vehicle/linkage deliberately."""
    if count not in (1, 3):
        raise ValueError("Only one particle or a transverse row of three")
    source = mujoco.MjModel.from_binary_path(str(bundle / "compiled_model.mjb"))
    xml = ET.parse(bundle / "model.xml").getroot()
    root = ET.Element("mujoco", model="isolated_bucket_entry")
    ET.SubElement(root, "compiler", angle="radian")
    option = copy.deepcopy(xml.find("option"))
    option.set("solver", "Newton")
    option.set("timestep", format(source.opt.timestep, ".17g"))
    root.append(option)
    root.append(copy.deepcopy(xml.find("asset")))
    world = ET.SubElement(root, "worldbody")
    floor = copy.deepcopy(xml.find("worldbody/geom"))
    world.append(floor)
    original = next(
        b for b in xml.iter("body") if b.get("name", "").endswith("_bucket")
    )
    bucket = ET.SubElement(world, "body", name="test_bucket")
    bucket.append(copy.deepcopy(original.find("inertial")))
    for name, kind, axis in [
        ("carriage_x", "slide", "1 0 0"),
        ("carriage_z", "slide", "0 0 1"),
        ("carriage_pitch", "hinge", "0 -1 0"),
    ]:
        ET.SubElement(bucket, "joint", name=name, type=kind, axis=axis)
    for geom in original.findall("geom"):
        bucket.append(copy.deepcopy(geom))
    grain = next(b for b in xml.iter("body") if b.get("name", "") == "grain_0")
    radius = source.geom_size[
        mujoco.mj_name2id(source, mujoco.mjtObj.mjOBJ_GEOM, "grain_geom_0"), 0
    ]
    for i in range(count):
        body = copy.deepcopy(grain)
        body.set("name", f"grain_{i}")
        body.set("pos", f"0.5 {(i - (count - 1) / 2) * 2.2 * radius} {radius + 0.001}")
        body.find("joint").set("name", f"grain_free_{i}")
        body.find("geom").set("name", f"grain_geom_{i}")
        world.append(body)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    # The preserved readable XML rounds some values. Restore authoritative compiled
    # collision/material arrays (including bounds) before running/saving the fixture.
    fields = [
        "size",
        "pos",
        "quat",
        "friction",
        "solref",
        "solimp",
        "margin",
        "gap",
        "solmix",
        "condim",
        "priority",
        "contype",
        "conaffinity",
        "rbound",
        "aabb",
    ]
    for g in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        name = "grain_geom_0" if name.startswith("grain_geom_") else name
        old = mujoco.mj_name2id(source, mujoco.mjtObj.mjOBJ_GEOM, name)
        for field in fields:
            getattr(model, "geom_" + field)[g] = getattr(source, "geom_" + field)[old]
    copied_bodies = [("test_bucket", original.get("name"))] + [
        (f"grain_{i}", "grain_0") for i in range(count)
    ]
    for new_name, old_name in copied_bodies:
        old = mujoco.mj_name2id(source, mujoco.mjtObj.mjOBJ_BODY, old_name)
        b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, new_name)
        for field in ["mass", "inertia", "ipos", "iquat"]:
            getattr(model, "body_" + field)[b] = getattr(source, "body_" + field)[old]
    data = mujoco.MjData(model)
    mujoco.mj_setConst(model, data)
    return model, ET.tostring(root, encoding="unicode")


def carriage_pose(t, curl_at, parameters):
    """Lip follows +X at 0.12 m/s; optional 0.25 rad curl about held bucket origin."""
    lip = np.array(
        [
            parameters["bucket"]["profile"][0][0],
            0,
            parameters["bucket"]["profile"][0][1],
        ]
    )
    edge = (
        np.asarray(parameters["bucket"]["profile"][0])
        - parameters["bucket"]["profile"][1]
    )
    edge_angle = np.arctan2(edge[1], edge[0])
    start_angle = -0.05 - edge_angle
    advance = 0.12 * np.clip(t - 1.0, 0, 14.0)
    # Curl trigger is carriage travel relative to initial particle front (0.5-r).
    onset = 1.0 + (0.5 - parameters["fixture_radius"] + curl_at) / 0.12
    curl = 0 if curl_at < 0 else np.clip((t - onset) * 0.20, 0, 0.25)
    angle = start_angle + curl

    def rotation(a):
        return np.array(
            [[np.cos(a), 0, -np.sin(a)], [0, 1, 0], [np.sin(a), 0, np.cos(a)]]
        )

    pos = np.array([advance, 0, 0.085]) - rotation(start_angle) @ lip
    quat = np.array([np.cos(angle / 2), 0, -np.sin(angle / 2), 0])
    return pos, quat


def run_case(bundle, output, count=1, curl_at=-1.0):
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use an empty output directory")
    output.mkdir(parents=True, exist_ok=True)
    model, xml = build_fixture(bundle, count)
    data = mujoco.MjData(model)
    parameters = load_parameters("configs/vehicle/wa320.yaml")
    grains = np.array(
        [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"grain_{i}")
            for i in range(count)
        ]
    )
    bucket = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "test_bucket")
    parameters["fixture_radius"] = float(model.geom_size[-1, 0])
    dt = model.opt.timestep
    n = round(16 / dt)
    records, contacts, accounting = [], [], []
    ledger = MaterialAccounting(model.body_mass[grains])
    funnel = AcquisitionFunnel(model.body_mass[grains], parameters)
    floor_clearance = []
    first_normal = None
    first_geom = None
    first_contact = None
    first_entry = None
    warnings = np.zeros(len(data.warning), dtype=int)
    # Only the initial carriage state is assigned. Subsequent motion uses bounded
    # generalized servo forces, never overwrites bucket/particle states.
    # Free particles are never moved/reset
    # after initialization, attached to the bucket, or assigned payload mass.
    pos0, quat0 = carriage_pose(0, curl_at, parameters)
    data.qpos[:3] = [pos0[0], pos0[2], -2 * np.arctan2(quat0[2], quat0[0])]
    tracking = []
    efforts = []
    mujoco.mj_forward(model, data)
    initial_xyz = data.xpos[grains].copy()
    radius = parameters["fixture_radius"]
    source_lo, source_hi = initial_xyz.min(0) - radius, initial_xyz.max(0) + radius
    for step in range(n):
        t = step * dt
        pos, quat = carriage_pose(t, curl_at, parameters)
        next_pos, next_quat = carriage_pose(t + dt, curl_at, parameters)
        target = np.array([pos[0], pos[2], -2 * np.arctan2(quat[2], quat[0])])
        next_target = np.array(
            [next_pos[0], next_pos[2], -2 * np.arctan2(next_quat[2], next_quat[0])]
        )
        error = target - data.qpos[:3]
        force = (
            data.qfrc_bias[:3]
            + np.array([2e5, 2e5, 1e5]) * error
            + np.array([4e4, 4e4, 2e4]) * ((next_target - target) / dt - data.qvel[:3])
        )
        data.qfrc_applied[:3] = np.clip(force, -3e5, 3e5)
        mujoco.mj_step(model, data)
        warnings = np.maximum(warnings, data.warning.number)
        if step % 8 and step != n - 1:
            continue
        mujoco.mj_forward(model, data)
        xyz = data.xpos[grains].copy()
        local = (xyz - data.xpos[bucket]) @ data.xmat[bucket].reshape(3, 3)
        inside = bucket_payload_mask(local, parameters)
        if inside.any() and first_entry is None:
            first_entry = t + dt
        acct = ledger.sample(
            inside,
            ((xyz >= source_lo) & (xyz <= source_hi)).all(1),
            np.zeros(count, bool),
        )
        accounting.append(list(acct.values()))
        tracking.append(error.copy())
        efforts.append(data.qfrc_applied[:3].copy())
        low = min(
            mujoco.mj_geomDistance(model, data, 0, int(g), 10.0, None)
            for g in np.flatnonzero(model.geom_bodyid == bucket)
        )
        floor_clearance.append(low)
        contacting = np.zeros(count, bool)
        upward = horizontal = 0.0
        for ci, contact in enumerate(data.contact[: data.ncon]):
            a, b = model.geom_bodyid[contact.geom]
            if (
                contact.efc_address < 0
                or bucket not in (a, b)
                or not any(g in (a, b) for g in grains)
            ):
                continue
            contacting[np.flatnonzero(grains == (b if a == bucket else a))] = True
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, ci, force)
            sign = 1 if a == bucket else -1
            normal = sign * contact.frame[:3]
            world_force = sign * (contact.frame.reshape(3, 3).T @ force[:3])
            contacts.append(
                [t + dt, ci, *contact.pos, *normal, *world_force, contact.dist]
            )
            upward += world_force[2]
            horizontal += world_force[0]
            if first_contact is None:
                first_contact = t + dt
                first_normal = normal.tolist()
                first_geom = mujoco.mj_id2name(
                    model,
                    mujoco.mjtObj.mjOBJ_GEOM,
                    int(contact.geom[0 if a == bucket else 1]),
                )
        funnel.sample(local, inside, contacting, "carriage_entry", t + dt)
        records.append(
            (
                t + dt,
                data.qpos.copy(),
                data.xpos[bucket].copy(),
                data.xquat[bucket].copy(),
                xyz,
                inside,
                upward,
                horizontal,
            )
        )
    time, qpos, pos, quat, xyz, inside, fz, fx = map(np.asarray, zip(*records))
    np.savez_compressed(
        output / "states.npz",
        time_s=time,
        tracking_error=tracking,
        carriage_effort=efforts,
        qpos=qpos,
        bucket_pos=pos,
        bucket_quat=quat,
        bucket_ground_clearance_m=floor_clearance,
        xyz=xyz,
        inside=inside,
        force_z_N=fz,
        force_x_N=fx,
        contacts=np.asarray(contacts).reshape(-1, 12),
        accounting=accounting,
        accounting_columns=list(acct),
    )
    mujoco.mj_saveModel(model, str(output / "compiled_model.mjb"))
    (output / "model.xml").write_text(xml)
    report = {
        "scope": "Finite-force three-axis bucket carriage, not full WA320 dynamics or cylinder actuation",
        "maximum_carriage_tracking_error_m_m_rad": abs(np.array(tracking))
        .max(0)
        .tolist(),
        "peak_carriage_effort_N_N_Nm": abs(np.array(efforts)).max(0).tolist(),
        "duration_s": n * dt,
        "timestep_s": dt,
        "observation_hz": 1 / (8 * dt),
        "carriage_servo_Kp_N_per_m_Nm_per_rad": [2e5, 2e5, 1e5],
        "carriage_servo_Kd_Ns_per_m_Nms_per_rad": [4e4, 4e4, 2e4],
        "carriage_generalized_force_limits_N_N_Nm": [3e5, 3e5, 3e5],
        "particle_count": count,
        "particle_radius_m": parameters["fixture_radius"],
        "initial_mass_kg": ledger.initial,
        "curl_start_entry_m": curl_at if curl_at >= 0 else None,
        "first_bucket_contact_s": first_contact,
        "first_contact_normal_on_particle_world": first_normal,
        "first_bucket_contact_geom": first_geom,
        "mouth_entry_events": funnel.crossings_in,
        "verified_acquired_count": int(funnel.acquired.sum()),
        "mouth_crossing_events": funnel.events,
        "minimum_bucket_floor_clearance_m": float(min(floor_clearance)),
        "maximum_particle_floor_overlap_m": float(
            max(0, parameters["fixture_radius"] - xyz[:, :, 2].min())
        ),
        "sampled_forces_finite": bool(
            np.isfinite(contacts).all() and np.isfinite(efforts).all()
        ),
        "first_inside_observation_s": first_entry,
        "peak_inside_count": int(inside.sum(1).max()),
        "final_inside_count": int(inside[-1].sum()),
        "final_accounting": acct,
        "maximum_accounting_error_kg": float(
            abs(
                np.asarray(accounting)[:, list(acct).index("conservation_residual_kg")]
            ).max()
        ),
        "peak_upward_bucket_force_on_particles_N": float(fz.max()),
        "peak_forward_bucket_force_on_particles_N": float(fx.max()),
        "max_particle_height_m": float(xyz[:, :, 2].max()),
        "particle_forward_displacement_m": (xyz[-1, :, 0] - xyz[0, :, 0]).tolist(),
        "solver_warnings": warnings.tolist(),
        "source_compiled_sha256": hashlib.sha256(
            (bundle / "compiled_model.mjb").read_bytes()
        ).hexdigest(),
    }
    (output / "report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    return report
