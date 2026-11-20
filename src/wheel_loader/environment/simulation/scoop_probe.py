"""M2 native MuJoCo experiment using the unchanged Newton-exported v1 loader.

Coarse spherical free rigid grains share one contact/dynamics solve with the
loader. This extends existing CPU material/mechanics diagnostics, not training.
"""

import hashlib
import json
import resource
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from wheel_loader.control.cylinder_actuation import (
    NAMES,
    FirstOrderCylinders,
    joint_efforts,
)
from wheel_loader.control.scripted_scoop import ScriptedScoop
from wheel_loader.evaluation.material_accounting import (
    AcquisitionFunnel,
    MaterialAccounting,
    bucket_payload_mask,
)
from wheel_loader.evaluation.pile_geometry import sampled_swept_volume, settled_envelope
from wheel_loader.terrain.bed import ScoopPile
from wheel_loader.vehicle.kinematics import passive_joint_positions, solve_linkage


def run_scoop(p, config, source, output, scenario=None):
    import mujoco

    wall_started = time.monotonic()
    cpu_started = time.process_time()
    scenario = scenario or config.get("default_scenario", "nominal")
    provenance = json.loads((Path(source).parent / "validation.json").read_text())
    parameter_digest = hashlib.sha256(
        json.dumps(p, sort_keys=True).encode()
    ).hexdigest()
    if provenance.get("parameter_sha256") != parameter_digest or not all(
        provenance.get("checks", {}).values()
    ):
        raise ValueError(
            "M2 requires a passing v1 source validation with matching vehicle parameters"
        )
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "report.json").unlink(missing_ok=True)
    case = dict(config["scenarios"][scenario])
    controller_cfg = dict(config["controller"])
    for key in (
        "attack_angle_deg",
        "attack_lip_height_m",
        "penetration_depth_m",
        "demonstrate_spillage",
    ):
        if key in case:
            controller_cfg[key] = case.pop(key)
    if not (
        config["settle_s"] >= 4
        and config["max_duration_s"] > config["settle_s"]
        and 0 < config["sample_hz"] <= 240
        and np.isfinite(config.get("wall_budget_s", 900))
        and config.get("wall_budget_s", 900) > 0
        and isinstance(config.get("granular_memory_mb", 512), int)
        and config.get("granular_memory_mb", 512) > 0
    ):
        raise ValueError("Invalid M2 experiment timing")
    material = dict(config["material"])
    material["particle_density_kg_m3"] = case.pop(
        "density_kg_m3", material["particle_density_kg_m3"]
    )
    if not all(np.isfinite(x) and x > 0 for x in material.values()):
        raise ValueError("Material trial values must be finite and positive")
    particle_budget = case.pop("particle_budget", config["max_particles"])
    pile = ScoopPile(
        **case,
        radius_m=config["radius_m"],
        seed=config["seed"],
        max_particles=particle_budget,
        particle_density_kg_m3=material["particle_density_kg_m3"],
    )
    xyz, radii = pile.generate()
    (output / "preparation.json").write_text(
        json.dumps(
            {
                "scenario": scenario,
                "prepared_particle_count": len(radii),
                "prepared_material_mass_kg": float(
                    np.sum(
                        4 / 3 * np.pi * radii**3 * material["particle_density_kg_m3"]
                    )
                ),
                "requested_pile_volume_m3": float(pile.volume_m3),
                "material_design": pile.material_design(),
                "pile_preparation": vars(pile),
                "status": "PREPARED_ONLY: settling and scoop not yet verified",
            },
            indent=2,
        )
        + "\n"
    )
    controller = ScriptedScoop(p, controller_cfg, config["settle_s"])
    lip = solve_linkage(p, *controller.attack_pose).lip + [
        0,
        p["vehicle"]["articulation_height"],
    ]
    xyz += [lip[0] + controller_cfg["approach_distance_m"] + 0.1, 0, 0.01]
    tree = ET.parse(source)
    root = tree.getroot()
    size = root.find("size")
    if size is None:
        size = ET.SubElement(root, "size")
    size.set("memory", f"{int(config.get('granular_memory_mb', 512))}M")
    world = root.find("worldbody")
    # Preserve the loader's complete body/contact/constraint model. Grains are
    # independent free bodies: no welds, kinematic trajectories or payload writes.
    for i, (point, radius) in enumerate(zip(xyz, radii)):
        body = ET.SubElement(
            world, "body", name=f"grain_{i}", pos=" ".join(map(str, point))
        )
        ET.SubElement(body, "freejoint", name=f"grain_free_{i}")
        ET.SubElement(
            body,
            "geom",
            name=f"grain_geom_{i}",
            type="sphere",
            size=str(radius),
            density=str(material["particle_density_kg_m3"]),
            friction=f"{material['sliding_friction']} {material['torsional_friction_m']} {material['rolling_friction_m']}",
            condim="6",
            priority="1",
            solref=f"{material['contact_time_s']} 1",
            solimp=".95 .99 .001",
            contype="1",
            conaffinity="-1",
            rgba=".66 .51 .31 1",
        )
    # Reuse the existing blade diagnostic's separate steel/grain contact trial.
    # Explicit pairs leave all vehicle-only contacts and geometry unchanged.
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    bucket_node = next(
        b for b in root.iter("body") if b.get("name", "").endswith("_bucket")
    )
    wall = config["bucket_contact"]
    friction = (
        [wall["sliding_friction"]] * 2
        + [wall["torsional_friction_m"]]
        + [wall["rolling_friction_m"]] * 2
    )
    for i in range(len(radii)):
        for geom in bucket_node.findall("geom"):
            ET.SubElement(
                contact,
                "pair",
                geom1=f"grain_geom_{i}",
                geom2=geom.get("name"),
                condim="6",
                friction=" ".join(map(str, friction)),
                solref=f"{material['contact_time_s']} 1",
                solimp=".95 .99 .001",
            )
    drives = root.find("actuator")
    if drives is None:
        drives = ET.SubElement(root, "actuator")
    drives.clear()
    joint_nodes = [j for j in root.iter("joint") if "_Physics_" in j.get("name", "")]
    for j in joint_nodes:
        name = j.get("name")
        if name.endswith(("wheel_joint", "_slider_joint")):
            ET.SubElement(drives, "motor", joint=name)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    compilation_wall_s = time.monotonic() - wall_started
    print(
        f"{scenario}: compiled {len(radii)} particles in {compilation_wall_s:.2f}s",
        flush=True,
    )
    dt = p["solver"]["dt"]
    model.opt.timestep = dt / 8
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.solver = {
        "Newton": mujoco.mjtSolver.mjSOL_NEWTON,
        "CG": mujoco.mjtSolver.mjSOL_CG,
    }[config["solver"]]
    model.opt.iterations = config["solver_iterations"]
    model.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY
    data = mujoco.MjData(model)
    joints = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j).split("_Physics_")[-1]: j
        for j in range(model.njnt)
        if "_Physics_" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or "")
    }
    names = list(joints)
    qa = np.array([model.jnt_qposadr[joints[n]] for n in names])
    va = np.array([model.jnt_dofadr[joints[n]] for n in names])
    bodies = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b): b
        for b in range(1, model.nbody)
    }
    find = lambda suffix: next(b for n, b in bodies.items() if n.endswith("_" + suffix))
    rear, front, bucket = map(find, ("rear_frame", "front_frame", "bucket"))
    vehicle_bodies = {b for n, b in bodies.items() if not n.startswith("grain_")}
    grain_bodies = np.array([bodies[f"grain_{i}"] for i in range(len(radii))])
    grain_set = set(grain_bodies)
    gj = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"grain_free_{i}")
        for i in range(len(radii))
    ]
    gqa = np.array([model.jnt_qposadr[j] for j in gj])
    free = next(
        j
        for j in range(model.njnt)
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE
        and model.jnt_bodyid[j] == rear
    )
    base = model.jnt_qposadr[free]
    data.qpos[base : base + 3] = [0, 0, p["vehicle"]["articulation_height"] + 0.02]
    initial_angles = p["maneuver"]["settle"]
    for n, value in passive_joint_positions(p, *initial_angles).items():
        data.qpos[qa[names.index(n)]] = value
    mujoco.mj_forward(model, data)
    actuator = FirstOrderCylinders(p, [0, *initial_angles])
    masses = model.body_mass[grain_bodies].copy()
    expected_masses = 4 / 3 * np.pi * radii**3 * material["particle_density_kg_m3"]
    if not np.allclose(masses, expected_masses, rtol=1e-12, atol=1e-12):
        raise ValueError("Compiled material mass differs from generation")
    if not np.isclose(model.body_mass[list(vehicle_bodies)].sum(), 15175):
        raise ValueError("M2 must preserve v1 vehicle mass")
    accounting = MaterialAccounting(masses)
    funnel = AcquisitionFunnel(masses, p)
    grain_index_by_body = np.full(model.nbody, -1, dtype=int)
    grain_index_by_body[grain_bodies] = np.arange(len(grain_bodies))
    wheels = [n for n in names if n.endswith("wheel_joint")]
    wheel_bodies = [int(model.jnt_bodyid[joints[n]]) for n in wheels]
    actuator_indices = [
        names.index(
            mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_JOINT, int(model.actuator_trnid[a, 0])
            ).split("_Physics_")[-1]
        )
        for a in range(model.nu)
    ]
    measured = [
        names.index(n + "_slider_joint")
        for n in ("steering_left", "lift_left", "bucket")
    ]
    neutral = np.array(
        [
            p["cylinder_bodies"][n]["reference_length"]
            for n in ("steering_left", "lift_left", "bucket")
        ]
    )
    sample_steps = round(1 / (config["sample_hz"] * dt))
    if not np.isclose(sample_steps * dt * config["sample_hz"], 1):
        raise ValueError("Sample period must divide control ticks")
    records, reports, phases = [], [], []
    (
        times,
        force_trace,
        stroke_trace,
        slip_trace,
        work_trace,
        pitch_trace,
        contact_trace,
    ) = [], [], [], [], [], [], []
    cylinder_names = list(p["cylinder_bodies"])
    cylinder_rods = [find(n + "_rod") for n in cylinder_names]
    cylinder_tips = [find(p["cylinder_bodies"][n]["tip_body"]) for n in cylinder_names]
    max_pin, max_zbar, max_stroke, max_self, max_overlap = 0.0, 0.0, 0.0, 0.0, 0.0
    max_bucket_ground_overlap = 0.0
    peak_traction, peak_bucket_force, work, positive_work = 0.0, 0.0, 0.0, 0.0
    bucket_force = 0.0
    bucket_touched = set()
    initial = None
    settled_hash = None
    settled_speed_rms = None
    initial_bucket_mass = None
    last_x, forward, reverse = 0.0, 0.0, 0.0
    started = time.monotonic()
    last_phase = None
    next_progress = 0.0
    folder_model = output / "model.xml"
    mujoco.mj_saveLastXML(str(folder_model), model)
    source_bounds = [xyz.min(axis=0) - radii.max(), xyz.max(axis=0) + radii.max()]
    stop_reason = None
    bucket_poses = []
    lip_trace = []
    contact_ids_by_bucket_geom = {}
    contact_peak_N_by_bucket_geom = {}
    bucket_resultant_trace = []
    peak_bucket_resultant_N = 0.0
    peak_contacts = 0
    for step in range(round(config["max_duration_s"] / dt)):
        t = step * dt
        actual, velocity = data.qpos[qa].copy(), data.qvel[va].copy()
        lengths = neutral + actual[measured]
        phase = controller.phase
        if phase != last_phase:
            print(f"{scenario}: {phase} at {t:.2f}s", flush=True)
            last_phase = phase
        u = controller.command(t, float(data.xpos[rear, 0]), lengths, bucket_force)
        effort, act = joint_efforts(p, actuator, u, actual, velocity, names, dt)
        data.ctrl[:] = effort[actuator_indices]
        for _ in range(8):
            mujoco.mj_step(model, data)
        power = float(effort @ data.qvel[va])
        work += power * dt
        # Positive part of net system power, not sum of positive actuator powers.
        positive_work += max(power, 0) * dt
        if (
            not np.isfinite(data.qpos).all()
            or not np.isfinite(data.qvel).all()
            or abs(data.qvel).max() > 1e4
        ):
            stop_reason = "nonfinite_or_explosive_state"
            break
        dx = float(data.xpos[rear, 0]) - last_x
        if t >= config["settle_s"]:
            forward += max(dx, 0)
            reverse += max(-dx, 0)
        last_x = float(data.xpos[rear, 0])
        # Observe every control tick, independently of the lower-rate recording.
        positions = data.xpos[grain_bodies].copy()
        local = (positions - data.xpos[bucket]) @ data.xmat[bucket].reshape(3, 3)
        in_bucket = bucket_payload_mask(local, p)
        contact_bodies = model.geom_bodyid[data.contact.geom[: data.ncon]]
        contacting = np.zeros(len(radii), bool)
        bucket_contacts = np.any(contact_bodies == bucket, axis=1)
        contact_grains = grain_index_by_body[contact_bodies[bucket_contacts].ravel()]
        contacting[contact_grains[contact_grains >= 0]] = True
        funnel.sample(local, in_bucket, contacting, phase, float(data.time))
        accounting.ever_acquired |= in_bucket
        peak_contacts = max(peak_contacts, data.ncon)
        if time.monotonic() - wall_started >= config.get("wall_budget_s", 900):
            stop_reason = "cpu_wall_budget_exhausted_incomplete_scoop"
        if step % sample_steps and not controller.done and stop_reason is None:
            continue
        mujoco.mj_forward(model, data)
        positions = data.xpos[grain_bodies].copy()
        if initial is None and t >= config["settle_s"]:
            initial = positions.copy()
            source_bounds = [
                positions.min(axis=0) - radii.max(),
                positions.max(axis=0) + radii.max(),
            ]
            gv = data.qvel[
                np.array([model.jnt_dofadr[j] for j in gj])[:, None] + np.arange(3)
            ]
            settled_speed_rms = float(np.sqrt(np.mean(np.sum(gv**2, axis=1))))
            settled_hash = hashlib.sha256(
                data.qpos.tobytes() + data.qvel.tobytes()
            ).hexdigest()
        local = (positions - data.xpos[bucket]) @ data.xmat[bucket].reshape(3, 3)
        in_bucket = bucket_payload_mask(local, p)
        lost = (
            ~np.isfinite(positions).all(axis=1)
            | (np.linalg.norm(positions[:, :2], axis=1) > 50)
            | (positions[:, 2] < -1)
        )
        at_source = (
            (positions >= source_bounds[0]) & (positions <= source_bounds[1])
        ).all(axis=1)
        record = accounting.sample(in_bucket, at_source, lost)
        if initial_bucket_mass is None and t >= config["settle_s"]:
            initial_bucket_mass = record["bucket_mass_kg"]
        bucket_force = 0.0
        bucket_resultant = np.zeros(3)
        traction = np.zeros(4)
        normals = np.zeros(4)
        for ci, c in enumerate(data.contact[: data.ncon]):
            bs = [int(model.geom_bodyid[g]) for g in c.geom]
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, ci, force)
            force_world = c.frame.reshape(3, 3).T @ force[:3]
            if any(b in grain_set for b in bs):
                max_overlap = max(max_overlap, -float(c.dist))
                if bucket in bs:
                    bucket_force += float(np.linalg.norm(force[:3]))
                    bucket_resultant += (1 if bs[1] == bucket else -1) * force_world
                    bucket_touched.update(b for b in bs if b in grain_set)
                    bucket_geom = next(
                        int(g) for g, b in zip(c.geom, bs) if b == bucket
                    )
                    name = mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_GEOM, bucket_geom
                    )
                    ids = [int(grain_index_by_body[b]) for b in bs if b in grain_set]
                    contact_ids_by_bucket_geom.setdefault(name, set()).update(ids)
                    contact_peak_N_by_bucket_geom[name] = max(
                        contact_peak_N_by_bucket_geom.get(name, 0),
                        float(np.linalg.norm(force[:3])),
                    )
            elif 0 in bs and bucket in bs:
                max_bucket_ground_overlap = max(
                    max_bucket_ground_overlap, -float(c.dist)
                )
            elif all(b in vehicle_bodies for b in bs):
                max_self = max(max_self, -float(c.dist))
            if 0 in bs or any(b in grain_set for b in bs):
                for k, b in enumerate(wheel_bodies):
                    if b in bs:
                        traction[k] += float(np.linalg.norm(force_world[:2]))
                        normals[k] += (1 if bs[1] == b else -1) * force_world[2]
        peak_bucket_force = max(peak_bucket_force, bucket_force)
        peak_bucket_resultant_N = max(
            peak_bucket_resultant_N, float(np.linalg.norm(bucket_resultant))
        )
        peak_traction = max(peak_traction, float(traction.max()))
        r = data.xmat[rear].reshape(3, 3)
        pitch = np.arcsin(np.clip(-r[2, 0], -1, 1))
        roll = np.arctan2(r[2, 1], r[2, 2])
        yaw = np.arctan2(r[1, 0], r[0, 0])
        slips = []
        for n, b in zip(wheels, wheel_bodies):
            vel = np.zeros(6)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, b, vel, 0)
            parent = rear if n.startswith("rear") else front
            slips.append(
                p["vehicle"]["wheel_radius"] * data.qvel[model.jnt_dofadr[joints[n]]]
                - vel[3:] @ data.xmat[parent].reshape(3, 3)[:, 0]
            )
        for name, rod, tip in zip(cylinder_names, cylinder_rods, cylinder_tips):
            spec = p["cylinder_bodies"][name]
            max_pin = max(
                max_pin,
                float(
                    np.linalg.norm(
                        data.xpos[rod]
                        - data.xpos[tip]
                        - data.xmat[tip].reshape(3, 3) @ spec["tip"]
                    )
                ),
            )
            c = p["cylinders"][spec["kind"]]
            extension = (
                data.qpos[qa[names.index(name + "_slider_joint")]]
                + spec["reference_length"]
                - c["pin_distance_min"]
            )
            max_stroke = max(max_stroke, -extension, extension - c["stroke"])
        rod = find("rod")
        anchor = np.array(
            [p["linkage"]["bucket_anchor"][0], 0, p["linkage"]["bucket_anchor"][1]]
        )
        end = np.array([p["linkage"]["rod_length"], 0, 0])
        max_zbar = max(
            max_zbar,
            float(
                np.linalg.norm(
                    data.xpos[rod]
                    + data.xmat[rod].reshape(3, 3) @ end
                    - data.xpos[bucket]
                    - data.xmat[bucket].reshape(3, 3) @ anchor
                )
            ),
        )
        if t >= next_progress:
            print(
                f"{scenario}: t={t:.1f}s x={data.xpos[rear, 0]:.3f} payload={record['bucket_mass_kg']:.2f}kg force={bucket_force:.0f}N",
                flush=True,
            )
            next_progress += 1
            (output / "runtime_progress.json").write_text(
                json.dumps(
                    {
                        "simulated_s": float(data.time),
                        "wall_s": time.monotonic() - wall_started,
                        "phase": phase,
                        "particle_count": len(radii),
                        "peak_process_rss_mb": resource.getrusage(
                            resource.RUSAGE_SELF
                        ).ru_maxrss
                        / 1024,
                        "peak_contacts": peak_contacts,
                        "solver_warning_count": int(data.warning.number.sum()),
                        "status": "IN_PROGRESS, not a completed scoop",
                    },
                    indent=2,
                )
                + "\n"
            )
        records.append(data.qpos.copy())
        rotation = data.xmat[bucket].reshape(3, 3).copy()
        if phase in ("penetrate", "curl_advance"):
            bucket_poses.append((data.xpos[bucket].copy(), rotation))
        edge = np.array([p["bucket"]["profile"][0][0], 0, p["bucket"]["profile"][0][1]])
        lip_trace.append(edge @ rotation.T + data.xpos[bucket])
        reports.append(record)
        times.append(float(data.time))
        phases.append(phase)
        force_trace.append(act["force_N"])
        stroke_trace.append(
            lengths - np.array([p["cylinders"][n]["pin_distance_min"] for n in NAMES])
        )
        slip_trace.append(slips)
        work_trace.append([work, positive_work])
        pitch_trace.append([pitch, roll, yaw])
        contact_trace.append([bucket_force, *traction, *normals])
        bucket_resultant_trace.append(bucket_resultant)
        if abs(pitch) > np.deg2rad(25) or abs(roll) > np.deg2rad(25):
            stop_reason = "vehicle_tilt_safety_stop"
            break
        if controller.done or stop_reason:
            break
    if stop_reason is None and not controller.done:
        stop_reason = "simulation_duration_exhausted_incomplete_scoop"
    if stop_reason is None and controller.failed:
        stop_reason = "controller_phase_timeout"
    if initial is None:
        initial = xyz
    positions = data.xpos[grain_bodies].copy()
    thresholds = config["acceptance"]
    retained = reports[-1]["bucket_mass_kg"] if reports else 0.0
    displaced = np.linalg.norm(positions - initial, axis=1)
    observed_entry = any(x["bucket_mass_kg"] > 0 for x in reports)
    residual = max(
        (abs(x["conservation_residual_kg"]) for x in reports), default=np.inf
    )
    retained_mask = in_bucket & ~lost
    payload_height = (
        float(positions[retained_mask, 2].min()) if retained_mask.any() else 0.0
    )
    payload_speed = (
        float(
            np.linalg.norm(
                data.qvel[
                    np.array([model.jnt_dofadr[j] for j in gj])[:, None] + np.arange(3)
                ],
                axis=1,
            )[retained_mask].max()
        )
        if retained_mask.any()
        else None
    )
    graph = {int(b): set() for b in grain_bodies}
    graph[bucket] = set()
    for contact in data.contact[: data.ncon]:
        a, b = [int(model.geom_bodyid[g]) for g in contact.geom]
        if a in graph and b in graph and contact.dist <= 0.001:
            graph[a].add(b)
            graph[b].add(a)
    supported = {bucket}
    pending = [bucket]
    while pending:
        for b in graph[pending.pop()] - supported:
            supported.add(b)
            pending.append(b)
    supported_payload = float(
        masses[
            retained_mask & np.array([int(b) in supported for b in grain_bodies])
        ].sum()
    )
    actual_initial_mass = float(masses.sum())
    envelope = settled_envelope(
        initial, radii, actual_initial_mass, material["particle_density_kg_m3"]
    )
    sweep = sampled_swept_volume(bucket_poses, p)
    acquisition = funnel.report(
        in_bucket & ~lost, at_source, initial, positions, radii, source_bounds
    )
    if initial_bucket_mass is None:
        envelope["status"] = (
            "UNSETTLED: CPU budget exhausted before settled initial state"
        )
    # Envelope and estimated sweep are geometric diagnostics, not a soil law.
    lips = np.asarray(lip_trace)
    source_lo = initial.min(axis=0) - radii.max()
    source_hi = initial.max(axis=0) + radii.max()
    relevant = np.array([phase in ("penetrate", "curl_advance") for phase in phases])
    cut = lips[relevant] if relevant.any() else None
    margins = {
        "method": "Global initial bounding-envelope margins; not local material occupancy beneath the cutting edge",
        "lateral_margin_outside_bucket_m": float(
            min(-source_lo[1], source_hi[1]) - p["vehicle"]["bucket_width"] / 2
        ),
        "bottom_material_below_cutting_edge_m": float(cut[:, 2].min() - source_lo[2])
        if cut is not None
        else None,
        "material_ahead_of_final_cutting_edge_m": float(source_hi[0] - cut[:, 0].max())
        if cut is not None
        else None,
        "full_nominal_particle_diameter_below_cutting_edge": bool(
            cut is not None and cut[:, 2].min() - source_lo[2] >= 2 * config["radius_m"]
        ),
        "full_scoop_observed": controller.done and stop_reason is None,
    }
    checks = {
        "pile_settled_before_approach": settled_speed_rms is not None
        and settled_speed_rms < 0.03,
        "initial_bucket_empty": initial_bucket_mass == 0,
        "sequence_completed": controller.done
        and not controller.failed
        and stop_reason is None,
        "physical_bucket_grain_contact": len(bucket_touched) > 0,
        "pile_deformation": bool(np.sum(displaced > 2 * radii) > 0),
        "material_entered_bucket": observed_entry,
        "positive_retained_payload": retained >= thresholds["retained_mass_min_kg"],
        "payload_clear_of_ground": payload_height > 2 * float(radii.max()),
        "payload_settled_after_reverse": payload_speed is not None
        and payload_speed < 0.05,
        "payload_physical_contact_support": retained > 0
        and abs(supported_payload - retained) < 1e-8,
        "spillage_accounted": all(
            x["bucket_exit_mass_kg"] >= 0
            and x["spilled_mass_kg"] >= x["bucket_exit_mass_kg"] - 1e-8
            for x in reports
        ),
        "mass_accounting": residual / accounting.initial
        <= thresholds["mass_relative_tolerance"],
        "no_boundary_loss": reports[-1]["boundary_loss_mass_kg"] == 0
        if reports
        else False,
        "cylinder_pin_closure": max_pin < thresholds["pin_gap_max_m"],
        "zbar_closure": max_zbar < thresholds["pin_gap_max_m"],
        "cylinder_strokes": max_stroke < thresholds["stroke_overrun_max_m"],
        "material_overlap": max_overlap
        <= thresholds["material_overlap_max_radius_fraction"] * float(radii.min()),
        "vehicle_stability": bool(
            len(pitch_trace)
            and np.max(abs(np.asarray(pitch_trace)[:, :2]))
            < np.deg2rad(thresholds["tilt_max_deg"])
        ),
        "bucket_ground_overlap": max_bucket_ground_overlap < 0.005,
        "vehicle_self_collision": max_self < 0.005,
        "no_solver_warnings": int(data.warning.number.sum()) == 0,
        "mouth_entry_explains_payload": not funnel.unverified_entry.any(),
        "physical_spillage_if_requested": not controller_cfg.get(
            "demonstrate_spillage", False
        )
        or acquisition["spilled_during_extraction_unique"]["mass_kg"]
        >= thresholds["spilled_mass_min_kg"],
    }
    checks = {name: bool(value) for name, value in checks.items()}
    result = {
        "scenario": scenario,
        "backend": "native MuJoCo CPU diagnostic of Newton-exported v1; NOT Isaac Lab/Newton GPU execution",
        "material_model": "coarse free rigid spheres with coupled six-dimensional frictional soft contact; uncalibrated",
        "native_contact_solver": f"{config['solver']}, elliptic cone, implicitfast, {config['solver_iterations']} iteration cap",
        "vehicle_commit": "97ae4e5",
        "experiment_sha256": hashlib.sha256(
            json.dumps(config, sort_keys=True).encode()
        ).hexdigest(),
        "physical_configuration_sha256": hashlib.sha256(
            json.dumps(
                {k: v for k, v in config.items() if k != "wall_budget_s"},
                sort_keys=True,
            ).encode()
        ).hexdigest(),
        "controlled_factors_sha256": hashlib.sha256(
            json.dumps(
                {
                    "vehicle": p,
                    "controller": controller_cfg,
                    "material": material,
                    "bucket_contact": wall,
                    "radius_m": config["radius_m"],
                    "seed": config["seed"],
                    "settle_s": config["settle_s"],
                    "solver": config["solver"],
                    "iterations": config["solver_iterations"],
                },
                sort_keys=True,
            ).encode()
        ).hexdigest(),
        "parameter_sha256": hashlib.sha256(
            json.dumps(p, sort_keys=True).encode()
        ).hexdigest(),
        "source_sha256": hashlib.sha256(Path(source).read_bytes()).hexdigest(),
        "initial_state_sha256": settled_hash,
        "settled_grain_speed_rms_m_s": settled_speed_rms,
        "initial_bucket_mass_kg": initial_bucket_mass,
        "prepared_envelope_height_m": float(
            np.max(xyz[:, 2] + radii) - np.min(xyz[:, 2] - radii)
        ),
        "settled_envelope_height_m": float(
            np.max(initial[:, 2] + radii) - np.min(initial[:, 2] - radii)
        ),
        "settled_envelope_width_m": float(
            np.max(initial[:, 1] + radii) - np.min(initial[:, 1] - radii)
        ),
        "settled_envelope_depth_m": float(
            np.max(initial[:, 0] + radii) - np.min(initial[:, 0] - radii)
        ),
        "material_parameters": material,
        "bucket_contact_parameters": wall,
        "controller": controller_cfg,
        "pile_preparation": vars(pile),
        "pile_material_design": pile.material_design(),
        "requested_pile_volume_m3": float(pile.volume_m3),
        "initial_pile_envelope": envelope,
        "initial_material_mass_kg": actual_initial_mass,
        "settled_initial_material_mass_kg": actual_initial_mass
        if initial_bucket_mass is not None
        else None,
        "mass_equations": "N * mean_compiled_particle_mass = actual_initial_material_mass; effective_bulk_density = actual_initial_material_mass / measured_convex_envelope_volume",
        "mean_particle_mass_kg": float(masses.mean()),
        "particle_mass_range_kg": [float(masses.min()), float(masses.max())],
        "retained_particle_count": int(retained_mask.sum()),
        "bucket_swept_volume_diagnostic": sweep,
        "bucket_swept_volume_m3": sweep["volume_m3"] if sweep else None,
        "bucket_swept_volume_over_initial_pile_volume": sweep["volume_m3"]
        / envelope["volume_m3"]
        if sweep
        else None,
        "pile_over_swept_volume_ratio": envelope["volume_m3"] / sweep["volume_m3"]
        if sweep and sweep["volume_m3"]
        else None,
        "reservoir_margins": margins,
        "acquisition_funnel": acquisition,
        "bucket_contact_diagnostics": {
            name: {
                "unique_contacting_count_20hz": len(ids),
                "unique_contacting_mass_kg_20hz": float(masses[list(ids)].sum()),
                "peak_contact_N_20hz": contact_peak_N_by_bucket_geom[name],
            }
            for name, ids in contact_ids_by_bucket_geom.items()
        },
        "element_count": len(radii),
        "particle_mass_kg": float(masses.sum()),
        "retained_payload_kg": retained,
        "final_retained_payload_kg": retained if checks["sequence_completed"] else None,
        "last_observed_bucket_mass_kg": retained,
        "payload_minimum_world_height_m": payload_height,
        "payload_maximum_final_speed_m_s": payload_speed,
        "payload_contact_connected_mass_kg": supported_payload,
        "peak_bucket_payload_kg": max(
            (x["bucket_mass_kg"] for x in reports), default=0
        ),
        "pile_mass_change_kg": accounting.initial - reports[-1]["remaining_mass_kg"]
        if reports
        else None,
        "peak_measured_bucket_exit_mass_kg": max(
            (x["bucket_exit_mass_kg"] for x in reports), default=0
        ),
        "solid_volume_fill_factor": retained / material["particle_density_kg_m3"] / 3.0,
        "uncleared_profile_extrusion_volume_m3": float(
            abs(
                np.sum(
                    np.asarray(p["bucket"]["profile"])[:, 0]
                    * np.roll(np.asarray(p["bucket"]["profile"])[:, 1], -1)
                    - np.asarray(p["bucket"]["profile"])[:, 1]
                    * np.roll(np.asarray(p["bucket"]["profile"])[:, 0], -1)
                )
            )
            / 2
            * p["vehicle"]["bucket_width"]
        ),
        "retained_bulk_equivalent_volume_m3": retained
        / envelope["effective_bulk_density_kg_m3"],
        "bulk_equivalent_fill_factor": retained
        / envelope["effective_bulk_density_kg_m3"]
        / 3.0,
        "nominal_bucket_capacity_m3": 3.0,
        "cycle_time_s": float(data.time),
        "runtime_wall_s": time.monotonic() - started,
        "compilation_and_preparation_wall_s": compilation_wall_s,
        "total_wall_s": time.monotonic() - wall_started,
        "process_cpu_s": time.process_time() - cpu_started,
        "simulation_seconds_per_cpu_second": float(
            data.time / (time.process_time() - cpu_started)
        ),
        "simulation_seconds_per_wall_second": float(
            data.time / (time.monotonic() - started)
        ),
        "peak_process_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        / 1024,
        "peak_contact_count": peak_contacts,
        "solver_arena_bytes": int(model.narena),
        "solver_warnings": {
            mujoco.mjtWarning(i).name: int(w.number)
            for i, w in enumerate(data.warning)
            if w.number
        },
        "forward_distance_m": forward,
        "reverse_distance_m": reverse,
        "peak_wheel_traction_N": peak_traction,
        "maximum_abs_wheel_slip_m_s": float(np.max(abs(np.asarray(slip_trace)))),
        "peak_chassis_tilt_deg": float(
            np.rad2deg(np.max(abs(np.asarray(pitch_trace)[:, :2])))
        ),
        "cylinder_extension_range_m": np.stack(
            [np.min(stroke_trace, axis=0), np.max(stroke_trace, axis=0)]
        ).tolist(),
        "nominal_cylinder_strokes_m": [p["cylinders"][n]["stroke"] for n in NAMES],
        "prepared_state_sha256": hashlib.sha256(
            xyz.tobytes() + radii.tobytes()
        ).hexdigest(),
        "peak_bucket_contact_force_N": peak_bucket_force,
        "bucket_contact_force_definition": "sum of grain-contact force magnitudes sampled at logging ticks; controller threshold proxy, not vector resultant",
        "peak_bucket_grain_resultant_force_N": peak_bucket_resultant_N,
        "peak_cylinder_force_N": np.max(abs(np.asarray(force_trace)), axis=0).tolist()
        if force_trace
        else None,
        "net_actuator_work_J": work,
        "positive_actuator_work_J": positive_work,
        "work_interpretation": "mechanical effort times joint speed, including wheel motors; not fuel/engine energy",
        "maximum_cylinder_pin_gap_m": max_pin,
        "maximum_zbar_gap_m": max_zbar,
        "maximum_stroke_overrun_m": max_stroke,
        "maximum_bucket_ground_overlap_m": max_bucket_ground_overlap,
        "maximum_vehicle_self_overlap_m": max_self,
        "maximum_material_overlap_m": max_overlap,
        "maximum_mass_residual_kg": residual,
        "final_accounting": reports[-1] if reports else {},
        "material_ids_displaced_over_diameter": int(np.sum(displaced > 2 * radii)),
        "material_mass_displaced_over_diameter_kg": float(
            masses[displaced > 2 * radii].sum()
        ),
        "pile_deformation_rms_displacement_m": float(np.sqrt(np.mean(displaced**2))),
        "pile_deformation_max_displacement_m": float(displaced.max()),
        "observed_penetration_lip_travel_m": float(cut[-1, 0] - cut[0, 0])
        if cut is not None
        else None,
        "events": controller.events,
        "stop_reason": stop_reason,
        "checks": checks,
        "single_run_passed": all(checks.values()),
        "m2_complete": False,
        "remaining_gate": "repeat run and scenario suite required; GPU coupling and material calibration not established",
    }
    (output / "accounting.json").write_text(json.dumps(reports, indent=2) + "\n")
    np.savez_compressed(
        output / "recording.npz",
        time_s=times,
        qpos=records,
        phase=phases,
        joint_names=names,
        joint_qpos_addresses=qa,
        grain_qpos_addresses=gqa,
        grain_body_ids=grain_bodies,
        radii_m=radii,
        masses_kg=masses,
        initial_pile_xyz=initial,
        final_pile_xyz=positions,
        accounting_columns=list(reports[0]) if reports else [],
        accounting=np.array([list(x.values()) for x in reports]),
        cylinder_force_N=force_trace,
        cylinder_extension_m=stroke_trace,
        wheel_slip_m_s=slip_trace,
        actuator_work_J=work_trace,
        chassis_angles_rad=pitch_trace,
        contact_N=contact_trace,
        bucket_grain_resultant_N_xyz=bucket_resultant_trace,
        bucket_lip_world_m=lip_trace,
        ever_bucket_contact=funnel.contacted,
        ever_mouth_entry=funnel.entered,
        ever_acquired=funnel.acquired,
        ever_mouth_exit=funnel.exited,
    )
    temporary_report = output / "report.json.tmp"
    temporary_report.write_text(json.dumps(result, indent=2) + "\n")
    temporary_report.replace(output / "report.json")
    print(
        json.dumps(
            {
                k: result[k]
                for k in (
                    "scenario",
                    "element_count",
                    "retained_payload_kg",
                    "cycle_time_s",
                    "runtime_wall_s",
                    "checks",
                )
            },
            indent=2,
        ),
        flush=True,
    )
    return result
