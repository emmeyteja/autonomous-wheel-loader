"""Native CPU dynamics diagnostics of the Newton-exported v1 model.

This independently steps MuJoCo; it is not the production Newton training path.
A retained rigid ballast tests load response/transfer, not granular material.
"""

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from wheel_loader.vehicle.kinematics import passive_joint_positions

CASES = (
    "acceleration",
    "braking",
    "articulation",
    "boom_raise",
    "bucket_curl",
    "carry_ballast",
    "transfer_ballast",
    "drive_lift",
    "rigid_terrain_contact",
    "static_lowered",
    "static_transport",
    "static_raised",
    "static_curled",
    "static_payload",
    "chassis_zero",
    "chassis_pos20",
    "chassis_neg20",
    "chassis_poslimit",
    "chassis_neglimit",
)


def run_probes(p, source, output, *, cases=None):
    import mujoco

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    baseline = mujoco.MjModel.from_xml_path(str(source))
    body_names = {
        mujoco.mj_id2name(baseline, mujoco.mjtObj.mjOBJ_BODY, i): i
        for i in range(baseline.nbody)
    }
    rear = next(n for n in body_names if n.endswith("_rear_frame"))
    bucket = next(n for n in body_names if n.endswith("_bucket"))
    all_reports = {}
    for case in CASES if cases is None else cases:
        tree = ET.parse(source)
        root = tree.getroot()
        has_payload = case.endswith("ballast") or case == "static_payload"
        mass = 2000.0 if has_payload else 0.0
        if has_payload:
            body = ET.SubElement(
                root.find("worldbody"), "body", name="diagnostic_ballast", pos="0 0 2"
            )
            ET.SubElement(body, "freejoint", name="ballast_free")
            ET.SubElement(
                body,
                "geom",
                name="ballast_collision",
                type="box",
                size=".25 .35 .20",
                mass=str(mass),
                priority="1",
                solref=".004 1",
                solimp=".99 .999 .001",
                rgba=".9 .3 .1 1",
                contype="1",
                conaffinity="2147483647",
            )
            ET.SubElement(
                root.find("equality"),
                "weld",
                name="ballast_attachment",
                body1=bucket,
                body2="diagnostic_ballast",
                relpose=".55 0 .30 1 0 0 0",
                solref=".01 1",
            )
            if case == "transfer_ballast":
                ET.SubElement(
                    root.find("worldbody"),
                    "geom",
                    name="receiver",
                    type="box",
                    pos="3.7 0 .15",
                    size="2 2 .1",
                    contype="1",
                    conaffinity="2147483647",
                    rgba=".3 .4 .5 1",
                )
        if case == "rigid_terrain_contact":
            from wheel_loader.terrain.rigid_pile import load_terrain
            from wheel_loader.vehicle.kinematics import solve_linkage

            terrain = load_terrain("configs/terrain/rigid_pile.yaml")
            toe = solve_linkage(p, *p["maneuver"]["lower_attack"]).lip[0] + 0.3
            vertices = terrain.vertices() + [toe + terrain.length / 2, 0, 0]
            ET.SubElement(
                root.find("asset"),
                "mesh",
                name="mechanical_pile",
                vertex=" ".join(map(str, vertices.ravel())),
            )
            ET.SubElement(
                root.find("worldbody"),
                "geom",
                name="mechanical_pile",
                type="mesh",
                mesh="mechanical_pile",
                rgba=".5 .4 .25 1",
                contype="1",
                conaffinity="2147483647",
            )
        drives = root.find("actuator")
        if drives is None:
            drives = ET.SubElement(root, "actuator")
            driven_names = [
                j.attrib["name"]
                for j in root.iter("joint")
                if "_Physics_" in j.attrib.get("name", "")
                and not j.attrib["name"].endswith(
                    ("bucket_joint", "rod_joint", "_hinge_joint")
                )
            ]
        else:
            driven_names = list(dict.fromkeys(node.attrib["joint"] for node in drives))
            drives.clear()
        for name in driven_names:
            ET.SubElement(drives, "motor", joint=name)
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        model.opt.timestep = p["solver"]["dt"] / 8
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        model.opt.iterations = 200
        model.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY
        data = mujoco.MjData(model)
        names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j).split("_Physics_")[
                -1
            ]: j
            for j in range(model.njnt)
        }
        addresses = {n: model.jnt_qposadr[j] for n, j in names.items()}
        for name, j in names.items():
            section = (
                "wheels"
                if name.endswith("wheel_joint")
                else {
                    "articulation_joint": "articulation",
                    "boom_joint": "boom",
                    "tilt_joint": "linkage",
                }.get(name)
            )
            if section:
                model.jnt_actfrclimited[j] = True
                model.jnt_actfrcrange[j] = [
                    -p[section]["max_torque"],
                    p[section]["max_torque"],
                ]
        rear_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, rear)
        bucket_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bucket)
        free = int(np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)[0])
        base = model.jnt_qposadr[free]
        data.qpos[base : base + 3] = [0, 0, p["vehicle"]["articulation_height"] + 0.02]
        current = np.array([0.15, -0.05])
        if case in ("bucket_curl", "carry_ballast"):
            current = np.array([0.45, 0.0])
        pose_cases = {
            "static_lowered": "lower_attack",
            "static_transport": "settle",
            "static_raised": "raise_dump",
            "static_curled": "curl_lift",
            "static_payload": "curl_lift",
            "rigid_terrain_contact": "lower_attack",
        }
        if case in pose_cases:
            current = np.array(p["maneuver"][pose_cases[case]])
        art0 = {
            "chassis_zero": 0.0,
            "chassis_pos20": np.deg2rad(20),
            "chassis_neg20": -np.deg2rad(20),
            "chassis_poslimit": p["articulation"]["max_angle"],
            "chassis_neglimit": p["articulation"]["min_angle"],
        }.get(case, 0.0)
        data.qpos[addresses["articulation_joint"]] = art0
        for n, v in passive_joint_positions(p, *current, articulation=art0).items():
            data.qpos[addresses[n]] = v
        mujoco.mj_forward(model, data)
        if has_payload:
            pj = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "ballast_free")
            pb = model.jnt_qposadr[pj]
            data.qpos[pb : pb + 3] = data.xpos[bucket_id] + data.xmat[
                bucket_id
            ].reshape(3, 3) @ [0.55, 0, 0.30]
            data.qpos[pb + 3 : pb + 7] = data.xquat[bucket_id]
            mujoco.mj_forward(model, data)
            eq = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_EQUALITY, "ballast_attachment"
            )
        wheel_ids = [names[n] for n in names if n.endswith("wheel_joint")]
        wheel_bodies = [int(model.jnt_bodyid[j]) for j in wheel_ids]
        front_id = next(i for n, i in body_names.items() if n.endswith("_front_frame"))
        duration = (
            34 if case == "transfer_ballast" else 20 if case == "boom_raise" else 12
        )
        from wheel_loader.control.cylinder_actuation import FirstOrderCylinders

        actuator = FirstOrderCylinders(p, [art0, *current])
        frames = []
        qstates = []
        metrics = []
        payload_states = []
        released = False
        receiver_contact = False
        telemetry = []
        actuator_rows = []
        work = 0.0
        positive_work = 0.0
        pile_contact = False
        max_pile_penetration = 0.0
        peak_pile_force = 0.0
        maximum_payload_penetration = 0.0
        maximum_self_penetration = 0.0
        self_pairs = {}
        cylinder_pin_gaps = []
        frame_translation_errors = []
        initial_energy = float(data.energy.sum())
        for step in range(round(duration / p["solver"]["dt"])):
            t = step * p["solver"]["dt"]
            speed = 0.0
            art = art0
            target = current.copy()
            if case in ("acceleration", "braking", "carry_ballast", "drive_lift"):
                vmax = 1.0 if case == "braking" else 0.6
                speed = (
                    vmax * min(max((t - 2) / 2, 0), 1) * min(max((10 - t) / 2, 0), 1)
                )
            if case == "articulation":
                speed = 0.4 if 2 < t < 10 else 0.0
                art = 0.45 * np.sin(np.clip((t - 2) / 8, 0, 1) * 2 * np.pi)
            if case == "rigid_terrain_contact":
                speed = 0.12 if 2 < t < 10 else 0.0
            if case == "drive_lift":
                target = np.array([0.60, 0.0])
            if case == "boom_raise":
                target = np.array([p["boom"]["max_angle"], 0.20])
            if case == "bucket_curl":
                target = np.array([0.45, -0.10 if t < 6 else 0.20])
            if case == "transfer_ballast":
                target = np.array(
                    [
                        p["boom"]["max_angle"],
                        0.20 if t < 19 else p["maneuver"]["dump"][1],
                    ]
                )
                if t >= 22 and not released:
                    data.eq_active[eq] = 0
                    released = True
            current += np.clip(
                target - current,
                -np.array([0.1, 0.22]) * p["solver"]["dt"],
                np.array([0.1, 0.22]) * p["solver"]["dt"],
            )
            from wheel_loader.control.commands import LoaderCommand
            from wheel_loader.control.cylinder_actuation import (
                joint_efforts,
                normalized_reference,
            )

            reference = LoaderCommand(speed, art, *current)
            control_names = list(names)
            actual = np.array([data.qpos[addresses[n]] for n in control_names])
            actual_vel = np.array(
                [data.qvel[model.jnt_dofadr[names[n]]] for n in control_names]
            )
            angles = [
                actual[control_names.index(n)]
                for n in ("articulation_joint", "boom_joint", "tilt_joint")
            ]
            u = normalized_reference(p, reference, angles)
            efforts, actuation = joint_efforts(
                p, actuator, u, actual, actual_vel, control_names, p["solver"]["dt"]
            )
            for a in range(model.nu):
                j = int(model.actuator_trnid[a, 0])
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j).split(
                    "_Physics_"
                )[-1]
                data.ctrl[a] = efforts[control_names.index(name)]
            for _ in range(8):
                mujoco.mj_step(model, data)
            power = float(
                efforts
                @ np.array(
                    [data.qvel[model.jnt_dofadr[names[n]]] for n in control_names]
                )
            )
            work += power * p["solver"]["dt"]
            positive_work += max(power, 0) * p["solver"]["dt"]
            if not np.isfinite(data.qpos).all() or np.max(np.abs(data.qvel)) > 1e4:
                raise RuntimeError(f"{case}: unstable state")
            if step % 12:
                continue
            mujoco.mj_forward(model, data)
            pin_gaps = []
            for name, spec in p.get("cylinder_bodies", {}).items():
                rod_body = next(
                    i for n, i in body_names.items() if n.endswith("_" + name + "_rod")
                )
                tip_body = next(
                    i
                    for n, i in body_names.items()
                    if n.endswith("_" + spec["tip_body"])
                )
                pin_gaps.append(
                    float(
                        np.linalg.norm(
                            data.xpos[rod_body]
                            - (
                                data.xpos[tip_body]
                                + data.xmat[tip_body].reshape(3, 3) @ spec["tip"]
                            )
                        )
                    )
                )
            cylinder_pin_gaps.append(max(pin_gaps, default=0.0))
            pile_force_sample = 0.0
            receiver_last_contact = False
            normal = np.zeros(4)
            slip = []
            for contact_index in range(data.ncon):
                contact = data.contact[contact_index]
                geoms = list(contact.geom)
                bodies = [int(model.geom_bodyid[g]) for g in geoms]
                labels = [
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
                    for g in geoms
                ]
                if "receiver" in labels and "ballast_collision" in labels:
                    receiver_contact = True
                    receiver_last_contact = True
                if "mechanical_pile" in labels:
                    if bucket_id in bodies:
                        pile_contact = True
                        force = np.zeros(6)
                        mujoco.mj_contactForce(model, data, contact_index, force)
                        pile_force_sample += float(np.linalg.norm(force[:3]))
                        peak_pile_force = max(peak_pile_force, pile_force_sample)
                    max_pile_penetration = max(
                        max_pile_penetration, -float(contact.dist)
                    )
                elif "ballast_collision" in labels:
                    maximum_payload_penetration = max(
                        maximum_payload_penetration, -float(contact.dist)
                    )
                elif 0 not in bodies:
                    maximum_self_penetration = max(
                        maximum_self_penetration, -float(contact.dist)
                    )
                    if contact.dist < -0.001:
                        pair = " | ".join(
                            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
                            for b in bodies
                        )
                        self_pairs[pair] = max(
                            self_pairs.get(pair, 0.0), -float(contact.dist)
                        )
                if 0 not in bodies:
                    continue
                f6 = np.zeros(6)
                mujoco.mj_contactForce(model, data, contact_index, f6)
                for k, b in enumerate(wheel_bodies):
                    if b in bodies:
                        normal[k] += (1 if bodies[1] == b else -1) * (
                            contact.frame.reshape(3, 3).T @ f6[:3]
                        )[2]
            for j, b in zip(wheel_ids, wheel_bodies):
                velocity = np.empty(6)
                mujoco.mj_objectVelocity(
                    model, data, mujoco.mjtObj.mjOBJ_BODY, b, velocity, 0
                )
                parent = (
                    rear_id
                    if "rear_"
                    in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "")
                    else front_id
                )
                axis = data.xmat[parent].reshape(3, 3)[:, 0]
                slip.append(
                    p["vehicle"]["wheel_radius"] * data.qvel[model.jnt_dofadr[j]]
                    - velocity[3:] @ axis
                )
            r = data.xmat[rear_id].reshape(3, 3)
            pitch = np.arcsin(np.clip(-r[2, 0], -1, 1))
            yaw = np.arctan2(r[1, 0], r[0, 0])
            metrics.append(
                [
                    t,
                    pitch,
                    yaw,
                    data.qpos[addresses["articulation_joint"]],
                    *normal,
                    *slip,
                ]
            )
            body_velocity = np.empty(6)
            mujoco.mj_objectVelocity(
                model, data, mujoco.mjtObj.mjOBJ_BODY, rear_id, body_velocity, 0
            )
            frame_translation_errors.append(
                float(np.linalg.norm(data.xpos[front_id] - data.xpos[rear_id]))
            )
            telemetry.append(
                [
                    *body_velocity[3:],
                    data.qpos[addresses["boom_joint"]],
                    data.qpos[addresses["bucket_joint"]],
                    power,
                    work,
                    positive_work,
                    float(data.energy.sum()),
                    mass,
                    pile_force_sample,
                ]
            )
            actuator_rows.append(actuation)
            frames.append(t)
            qstates.append(data.qpos.copy())
            payload_states.append(
                data.xpos[
                    mujoco.mj_name2id(
                        model, mujoco.mjtObj.mjOBJ_BODY, "diagnostic_ballast"
                    )
                ].copy()
                if has_payload
                else np.full(3, np.nan)
            )
        folder = output / case
        folder.mkdir(exist_ok=True)
        mujoco.mj_saveLastXML(str(folder / "model.xml"), model)
        values = np.array(metrics)
        checks = {
            "finite_states": bool(np.isfinite(qstates).all()),
            "positive_vehicle_mass": bool(
                np.isclose(model.body_mass.sum(), 15175 + mass, atol=0.1)
            ),
            "bounded_pitch": bool(np.max(abs(values[:, 1])) < 0.10),
            "bounded_articulation": bool(
                np.max(abs(values[:, 3])) < np.deg2rad(38.5) + 0.01
            ),
        }
        if case == "transfer_ballast":
            checks |= {
                "attachment_released": released,
                "ballast_contacts_receiver": receiver_contact,
                "ballast_remains_on_receiver": receiver_last_contact,
            }
        front_mask = np.array(
            [
                "front_"
                in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "")
                for b in wheel_bodies
            ]
        )
        front_force = values[:, 4:8][:, front_mask].sum(axis=1)
        rear_force = values[:, 4:8][:, ~front_mask].sum(axis=1)
        moving_speed = np.gradient(np.array(qstates)[:, base], np.array(frames))
        checks["vehicle_response"] = (
            bool(np.ptp(np.array(qstates)[:, base]) > 0.5)
            if case in ("acceleration", "braking", "carry_ballast", "drive_lift")
            else True
        )
        checks["sustained_ground_support"] = bool(
            np.mean(values[values[:, 0] > 2, 4:8].sum(axis=1) > 1000) > 0.95
        )
        if case.startswith(("static_", "chassis_")):
            final = values[values[:, 0] > duration - 2]
            means = final[:, 4:8].mean(axis=0)
            support_error = abs(means.sum() - (15175 + mass) * 9.81) / (
                (15175 + mass) * 9.81
            )
            checks.update(
                static_weight_balance=bool(support_error < 0.03),
                four_positive_wheel_loads=bool((means > 0).all()),
                stable_equilibrium=bool(
                    np.max(np.abs(np.array(telemetry)[values[:, 0] > duration - 2, :3]))
                    < 0.01
                ),
            )
        if has_payload:
            checks["payload_contact_depth"] = maximum_payload_penetration < 0.005
        if case.startswith("chassis_"):
            checks["requested_articulation"] = bool(abs(values[-1, 3] - art0) < 0.005)
        checks["self_collision"] = bool(maximum_self_penetration < 0.005)
        if case == "rigid_terrain_contact":
            checks.update(
                bucket_contacts_rigid_terrain=pile_contact,
                rigid_contact_depth=bool(max_pile_penetration < 0.02),
            )
        from wheel_loader.evaluation.actuation import check_actuation

        actuator_check = check_actuation(
            p,
            {k: np.array([row[k] for row in actuator_rows]) for k in actuator_rows[0]},
        )
        checks.update(actuator_check["checks"])
        stroke_overruns = {}
        for name, spec in p.get("cylinder_bodies", {}).items():
            x = (
                np.array(qstates)[:, addresses[name + "_slider_joint"]]
                + spec["reference_length"]
                - p["cylinders"][spec["kind"]]["pin_distance_min"]
            )
            stroke = p["cylinders"][spec["kind"]]["stroke"]
            stroke_overruns[name] = float(
                np.maximum(np.maximum(-x, x - stroke), 0).max()
            )
        checks["all_physical_slider_strokes"] = all(
            x < 0.001 for x in stroke_overruns.values()
        )
        checks["no_chassis_translation"] = max(frame_translation_errors) < 1e-5
        checks["cylinder_pin_closure"] = max(cylinder_pin_gaps) < 0.001

        static_prediction = None
        if case.startswith("static_"):
            from wheel_loader.vehicle.mass_properties import mass_report

            prediction = mass_report(
                p,
                *current,
                payload_kg=mass,
                payload_local_m=[0.55, 0, 0.30] if mass else None,
            )
            measured_front = float(
                values[values[:, 0] > duration - 2, 4:8][:, front_mask]
                .sum(axis=1)
                .mean()
            )
            error = abs(measured_front - prediction["front_axle_load_N"]) / (
                (15175 + mass) * 9.81
            )
            checks["static_front_axle_prediction"] = bool(error < 0.03)
            static_prediction = {
                "front_N": prediction["front_axle_load_N"],
                "rear_N": prediction["rear_axle_load_N"],
                "total_COM_m": prediction["total_vehicle_COM_m"],
                "front_fraction_error": error,
            }
        record = {
            "maximum_rigid_pile_overlap_m": max_pile_penetration,
            "peak_bucket_rigid_pile_force_N": peak_pile_force,
            "maximum_payload_contact_penetration_m": maximum_payload_penetration,
            "static_prediction": static_prediction,
            "actuation": actuator_check,
            "maximum_cylinder_pin_gap_m": max(cylinder_pin_gaps),
            "maximum_slider_stroke_overrun_m": stroke_overruns,
            "maximum_front_rear_hinge_translation_m": max(frame_translation_errors),
            "maximum_self_penetration_m": maximum_self_penetration,
            "self_pairs": self_pairs,
            "requested_articulation_rad": float(art0),
            "final_wheel_normal_N": values[values[:, 0] > duration - 2, 4:8]
            .mean(axis=0)
            .tolist(),
            "net_actuator_work_J": work,
            "positive_actuator_work_J": positive_work,
            "mechanical_energy_change_J": float(data.energy.sum()) - initial_energy,
            "energy_balance_interpretation": "Actuator work and mechanical energy only; contact/friction losses and valve/pump efficiency not identified.",
            "deformable_terrain_penetration": "NOT IMPLEMENTED: this case tests reaction against an immovable rigid pile"
            if case == "rigid_terrain_contact"
            else None,
            "front_axle_force_range_after_settle_N": [
                float(front_force[values[:, 0] > 2].min()),
                float(front_force[values[:, 0] > 2].max()),
            ],
            "rear_axle_force_range_after_settle_N": [
                float(rear_force[values[:, 0] > 2].min()),
                float(rear_force[values[:, 0] > 2].max()),
            ],
            "peak_forward_speed_m_s": float(moving_speed.max()),
            "final_forward_speed_m_s": float(moving_speed[-1]),
            "wheel_body_names": [
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
                for b in wheel_bodies
            ],
            "parameter_sha256": hashlib.sha256(
                json.dumps(p, sort_keys=True).encode()
            ).hexdigest(),
            "case": case,
            "backend": "native MuJoCo diagnostic of Newton-exported model",
            "checks": checks,
            "payload_kg": mass,
            "payload_model": "retained rigid ballast, not material"
            if has_payload
            else "none",
            "peak_pitch_deg": float(np.rad2deg(abs(values[:, 1]).max())),
            "yaw_range_deg": float(np.rad2deg(np.ptp(np.unwrap(values[:, 2])))),
            "maximum_abs_wheel_slip_m_s": float(abs(values[:, 8:]).max()),
            "minimum_total_wheel_normal_after_settle_N": float(
                values[values[:, 0] > 2, 4:8].sum(axis=1).min()
            ),
            "mass_conserved_kg": float(model.body_mass.sum()),
            "hydraulic_accuracy_validated": False,
        }
        (folder / "report.json").write_text(json.dumps(record, indent=2) + "\n")
        np.savez_compressed(
            folder / "dynamics.npz",
            time_s=frames,
            qpos=qstates,
            metrics=values,
            payload_xyz=payload_states,
            state_columns=np.array(
                [
                    "vx_m_s",
                    "vy_m_s",
                    "vz_m_s",
                    "boom_rad",
                    "bucket_rad",
                    "power_W",
                    "work_J",
                    "positive_work_J",
                    "mechanical_energy_J",
                    "payload_kg",
                    "bucket_rigid_contact_N",
                ]
            ),
            state=np.array(telemetry),
            **{
                k: np.array([row[k] for row in actuator_rows]) for k in actuator_rows[0]
            },
        )
        all_reports[case] = record
    (output / "suite.json").write_text(json.dumps(all_reports, indent=2) + "\n")
    return all_reports


def run_v_cycle_probe(p, source, output):
    """Full V-cycle with normalized force actuation, using the same exported geometry."""
    from dataclasses import asdict

    import mujoco

    from wheel_loader.control.cylinder_actuation import (
        FirstOrderCylinders,
        joint_efforts,
        normalized_reference,
    )
    from wheel_loader.control.v_cycle import PHASES, VCycleController, VCycleScenario
    from wheel_loader.evaluation.v_cycle import evaluate_v_cycle

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source)
    xml = tree.getroot()
    old = xml.find("actuator")
    if old is not None:
        xml.remove(old)
    motors = ET.SubElement(xml, "actuator")
    joint_names = [
        j.attrib["name"]
        for j in xml.iter("joint")
        if "_Physics_" in j.attrib.get("name", "")
    ]
    for name in joint_names:
        if not name.endswith(("bucket_joint", "rod_joint", "_hinge_joint")):
            ET.SubElement(motors, "motor", joint=name)
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    model.opt.timestep = p["solver"]["dt"] / 8
    data = mujoco.MjData(model)
    names = [n.split("_Physics_")[-1] for n in joint_names]
    joints = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in joint_names
    ]
    addresses = model.jnt_qposadr[joints]
    veladr = model.jnt_dofadr[joints]
    for name, j in zip(names, joints):
        section = (
            "wheels"
            if name.endswith("wheel_joint")
            else {
                "articulation_joint": "articulation",
                "boom_joint": "boom",
                "tilt_joint": "linkage",
            }.get(name)
        )
        if section:
            model.jnt_actfrclimited[j] = True
            model.jnt_actfrcrange[j] = [
                -p[section]["max_torque"],
                p[section]["max_torque"],
            ]
    free = int(np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)[0])
    base = model.jnt_qposadr[free]
    data.qpos[base : base + 3] = [0, 0, p["vehicle"]["articulation_height"] + 0.02]
    for name, v in passive_joint_positions(p, *p["maneuver"]["settle"]).items():
        data.qpos[addresses[names.index(name)]] = v
    rear = next(
        i
        for i in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").endswith(
            "_rear_frame"
        )
    )
    mujoco.mj_forward(model, data)
    scenario = VCycleScenario()
    cycle = VCycleController(p, scenario)
    actuator = FirstOrderCylinders(p, [0, *p["maneuver"]["settle"]])
    positions = []
    roots = []
    commands = []
    phases = []
    actuation = []
    qpos = []
    gaps = []
    rod_id = next(
        i
        for i in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").endswith(
            "_rod"
        )
    )
    bucket_id = next(
        i
        for i in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").endswith(
            "_bucket"
        )
    )
    for step in range(round(scenario.max_duration / p["solver"]["dt"])):
        t = step * p["solver"]["dt"]
        r = data.xmat[rear].reshape(3, 3)
        yaw = np.arctan2(r[1, 0], r[0, 0])
        phase = cycle.phase
        q = data.qpos[addresses].copy()
        qd = data.qvel[veladr].copy()
        reference = cycle.command(
            t,
            p["solver"]["dt"],
            data.xpos[rear, :2],
            float(yaw),
            float(q[names.index("articulation_joint")]),
            attachment=[q[names.index("boom_joint")], q[names.index("tilt_joint")]],
        )
        angles = [
            q[names.index(n)]
            for n in ("articulation_joint", "boom_joint", "tilt_joint")
        ]
        u = normalized_reference(p, reference, angles)
        effort, meta = joint_efforts(p, actuator, u, q, qd, names, p["solver"]["dt"])
        for a in range(model.nu):
            data.ctrl[a] = effort[joints.index(int(model.actuator_trnid[a, 0]))]
        mujoco.mj_step(model, data, nstep=8)
        if not np.isfinite(data.qpos).all():
            raise RuntimeError("Nonfinite full-cycle state")
        mujoco.mj_forward(model, data)
        rod_tip = data.xpos[rod_id] + data.xmat[rod_id].reshape(3, 3) @ [
            p["linkage"]["rod_length"],
            0,
            0,
        ]
        bx, bz = p["linkage"]["bucket_anchor"]
        bucket_tip = data.xpos[bucket_id] + data.xmat[bucket_id].reshape(3, 3) @ [
            bx,
            0,
            bz,
        ]
        frame_gaps = [float(np.linalg.norm(rod_tip - bucket_tip))]
        for name, spec in p.get("cylinder_bodies", {}).items():
            rid = next(
                i
                for i in range(model.nbody)
                if (
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
                ).endswith("_" + name + "_rod")
            )
            tid = next(
                i
                for i in range(model.nbody)
                if (
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
                ).endswith("_" + spec["tip_body"])
            )
            frame_gaps.append(
                float(
                    np.linalg.norm(
                        data.xpos[rid]
                        - data.xpos[tid]
                        - data.xmat[tid].reshape(3, 3) @ spec["tip"]
                    )
                )
            )
        gaps.append(max(frame_gaps))
        positions.append(data.qpos[addresses].copy())
        quat = data.qpos[base + 3 : base + 7]
        roots.append(np.r_[data.qpos[base : base + 3], quat[[1, 2, 3, 0]]])
        commands.append(
            [
                reference.rear_axle_speed,
                reference.articulation_angle,
                reference.boom_angle,
                reference.tilt_angle,
            ]
        )
        phases.append(phase)
        actuation.append(meta)
        qpos.append(data.qpos.copy())
        if cycle.phase != phase:
            print("CPU V-cycle:", PHASES[phase], round(t, 1), flush=True)
        if cycle.done:
            break
    result = evaluate_v_cycle(
        p,
        asdict(scenario) | {"joint_names": names},
        np.array(positions),
        np.array(roots),
        np.array(phases),
        np.array(commands),
        cycle.events,
        cycle.done and not cycle.timeout,
    )
    result["maximum_loop_gap_m"] = max(gaps)
    result["checks"]["physical_loop_closure"] = max(gaps) < 0.005
    actual = np.array([r["actual_length_m"] for r in actuation])
    lo = actuator.lo
    hi = actuator.hi
    overrun = np.maximum(np.maximum(lo - actual, actual - hi), 0)
    result["maximum_cylinder_length_overrun_m"] = overrun.max(axis=0).tolist()
    result["checks"]["physical_cylinder_lengths"] = bool(overrun.max() < 0.001)
    result["checks"]["bounded_reference_lengths"] = bool(
        np.all(np.array([r["reference_length_m"] for r in actuation]) >= lo)
        and np.all(np.array([r["reference_length_m"] for r in actuation]) <= hi)
    )
    from wheel_loader.evaluation.actuation import check_workspace

    result["workspace"] = check_workspace(
        p,
        {
            "joint_names": np.asarray(names),
            "positions": np.asarray(positions),
            "root_poses_xyzw": np.asarray(roots),
            "phase_ids": np.asarray(phases),
        },
    )
    result["checks"].update(result["workspace"]["checks"])
    result["parameter_sha256"] = hashlib.sha256(
        json.dumps(p, sort_keys=True).encode()
    ).hexdigest()
    result["backend"] = (
        "native MuJoCo full-cycle diagnostic; same Newton-exported v1 geometry"
    )
    result["actuator_model"] = (
        "normalized first-order force-limited cylinders and motor proxy; no state overrides after reset"
    )
    result["physical_accuracy"] = "estimated and uncalibrated"
    mujoco.mj_saveLastXML(str(output / "effective_model.xml"), model)
    np.savez_compressed(
        output / "motion.npz",
        positions=positions,
        root_poses_xyzw=roots,
        joint_names=names,
        time_s=np.arange(len(positions)) * p["solver"]["dt"],
        phase_ids=phases,
        phase_names=PHASES,
        commands=commands,
        loop_gap_m=gaps,
    )
    np.savez_compressed(
        output / "actuation.npz",
        **{k: np.array([r[k] for r in actuation]) for k in actuation[0]},
    )
    (output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    return result
