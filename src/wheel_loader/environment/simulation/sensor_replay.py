"""Offline CPU sensor diagnostic over recorded Newton/MuJoCo vehicle states.

No physics advancement, training backend or hardware connection is implemented here.
"""

import hashlib
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from wheel_loader.sensing.contracts import Availability, RangeScan
from wheel_loader.sensing.geometry import (
    HeightGrid,
    map_points,
    ray_directions,
    rotation_rpy_deg,
)
from wheel_loader.sensing.noise import RangeNoise
from wheel_loader.sensing.odometry import ArticulatedOdometry

from .replay_state import ReplayState, world_angular_velocity


def raycast(model, data, origin, directions, max_range):
    import mujoco

    directions = np.ascontiguousarray(directions, dtype=np.float64)
    ids = np.empty(len(directions), np.int32)
    ranges = np.empty(len(directions), np.float64)
    mujoco.mj_multiRay(
        model,
        data,
        np.asarray(origin, dtype=np.float64),
        directions.ravel(),
        None,
        True,
        -1,
        ids,
        ranges,
        None,
        len(directions),
        max_range,
    )
    ranges[(ids < 0) | (ranges < 0) | (ranges > max_range)] = np.nan
    return ranges, ids


def sample_indices(times, hz):
    dt = float(np.median(np.diff(times)))
    stride = round(1 / (dt * hz))
    if (
        stride < 1
        or not np.allclose(np.diff(times), dt, atol=1e-8)
        or not np.isclose(stride * dt * hz, 1, atol=1e-6)
    ):
        raise ValueError(
            "Sensor rate must divide recorded rate; no fabricated upsampling"
        )
    return np.arange(0, len(times), stride)


def run_sensor_replay(folder, output, parameters, config):
    import mujoco

    folder, output = Path(folder), Path(output)
    source_report = json.loads((folder / "validation.json").read_text())
    if not all(source_report["checks"].values()):
        raise ValueError("Use a source run that passed its own numerical checks")
    with np.load(folder / "motion.npz") as n:
        times = n["time_s"]
        positions = n["positions"]
        roots = n["root_poses_xyzw"]
        names = n["joint_names"].tolist()
        contact = n["bucket_contact_forces_w"]
    if len(contact) != len(times):
        raise ValueError("Source run has no aligned bucket contact channel")
    if (
        len(times) < 3
        or not np.isfinite(positions).all()
        or not np.isfinite(roots).all()
    ):
        raise ValueError("Invalid source states")
    e_idx = sample_indices(times, config["encoder_hz"])
    s_idx = sample_indices(times, config["lidar_hz"])
    if not np.isin(s_idx, e_idx).all():
        raise ValueError(
            "LiDAR capture times must align with encoder ticks in this diagnostic"
        )
    xml = folder / "effective_model.xml"
    model = mujoco.MjModel.from_xml_path(str(xml))
    replay = ReplayState(model, names, positions, roots, times, parameters)
    data = replay.data
    v = parameters["vehicle"]
    half = v["wheelbase"] / 2
    wheels = positions[e_idx][
        :, [names.index("rear_left_wheel_joint"), names.index("rear_right_wheel_joint")]
    ]
    art = positions[e_idx, names.index("articulation_joint")]
    estimator = ArticulatedOdometry(1, v["wheel_radius"], v["wheelbase"])
    odom = np.array(
        [estimator.update(w[None, :], np.array([a]))[0] for w, a in zip(wheels, art)]
    )
    # Privileged evaluation and IMU synthesis use recorded state; the odometry estimator above does not.
    truth = np.empty((len(e_idx), 3))
    body_rotations = []
    root_xyz = []
    rear = replay.bodies["rear_frame"]
    for k, i in enumerate(e_idx):
        replay.configuration(int(i))
        r = data.xmat[rear].reshape(3, 3).copy()
        body_rotations.append(r)
        root_xyz.append(data.xpos[rear].copy())
        axle = data.xpos[rear] + r @ [
            -half,
            0,
            v["wheel_radius"] - v["articulation_height"],
        ]
        truth[k] = [*axle[:2], np.arctan2(r[1, 0], r[0, 0])]
    # Align truth only for evaluation, never for estimator initialization/update.
    origin = truth[0, :2].copy()
    yaw0 = truth[0, 2]
    c, s = np.cos(yaw0), np.sin(yaw0)
    truth[:, :2] = (truth[:, :2] - origin) @ np.array([[c, -s], [s, c]])
    truth[:, 2] = np.unwrap(truth[:, 2]) - yaw0
    t = times[e_idx]
    root_xyz = np.asarray(root_xyz)
    rotations = np.asarray(body_rotations)
    accel = np.gradient(np.gradient(root_xyz, t, axis=0), t, axis=0)
    specific_force = np.einsum("nji,nj->ni", rotations, accel - [0, 0, -9.81])
    gyro = []
    for j in range(len(t)):
        lo, hi = max(0, j - 1), min(len(t) - 1, j + 1)
        gyro.append(
            rotations[j].T
            @ world_angular_velocity(rotations[lo], rotations[hi], t[hi] - t[lo])
        )
    imu_valid = np.ones(len(t), bool)
    imu_valid[[0, 1, -2, -1]] = False
    specific_force[~imu_valid] = np.nan
    gyro = np.asarray(gyro)
    gyro[~imu_valid] = np.nan
    spec = config["lidar"]
    dirs = ray_directions(
        spec["azimuth_deg"], spec["elevation_deg"], spec["columns"], spec["channels"]
    )
    rmount = rotation_rpy_deg(spec["rpy_deg"])
    tmount = np.array(spec["translation_m"])
    noise = RangeNoise(
        1,
        config["seed"],
        spec["range_noise_std_m"],
        spec["dropout_probability"],
        spec["min_range_m"],
        spec["max_range_m"],
    )
    grid = HeightGrid(
        config["map"]["x_bounds_m"],
        config["map"]["y_bounds_m"],
        config["map"]["cell_size_m"],
    )
    scans = []
    masks = []
    mapped = []
    origins = []
    world_rot = []
    geom_ids = []
    lookup = {int(i): j for j, i in enumerate(e_idx)}
    mount_body = replay.bodies[spec["mount_body"]]
    for i in s_idx:
        replay.configuration(int(i))
        rbody = data.xmat[mount_body].reshape(3, 3)
        rworld = rbody @ rmount
        ow = data.xpos[mount_body] + rbody @ tmount
        raw, ids = raycast(model, data, ow, dirs @ rworld.T, spec["max_range_m"])
        raw[raw < spec["min_range_m"]] = np.nan
        ranges, valid = noise.apply(raw[None, :])
        scan = RangeScan(
            np.array([times[i]]),
            np.array([times[i] + spec["latency_s"]]),
            ranges,
            valid,
            dirs,
        )
        points = map_points(
            scan.points_sensor_m(),
            odom[lookup[int(i)]][None, :],
            tmount,
            rmount,
            half,
            v["articulation_height"],
        )[0]
        grid.update(points)
        scans.append(scan.ranges_m[0])
        masks.append(scan.valid[0])
        mapped.append(points)
        origins.append(ow.copy())
        world_rot.append(rworld.copy())
        geom_ids.append(ids)
    scans, masks, mapped = np.asarray(scans), np.asarray(masks), np.asarray(mapped)
    drift = np.linalg.norm(odom[:, :2] - truth[:, :2], axis=1)
    channels = {
        "lidar": Availability(
            True,
            "range/point cloud",
            "m",
            "Native MuJoCo geometric raycasts over recorded Newton states; trial range noise, dropout and latency",
        ),
        "wheel_odometry": Availability(
            True,
            "rear axle XY/yaw",
            "m,rad",
            "Wheel angles + articulation only, ideal encoder data; equal axle offsets and no slip assumed",
        ),
        "joint_encoders": Availability(
            True,
            "relative joint displacement",
            "rad",
            "Ideal recorded joint positions; not calibrated instruments",
        ),
        "imu": Availability(
            True,
            "specific force/angular rate",
            "m/s^2,rad/s",
            "Finite differences of recorded simulator poses; no bias/noise calibration",
        ),
        "bucket_contact_force": Availability(
            True,
            "net contact force on bucket in world axes",
            "N",
            "Original Newton contact sensor; NOT hydraulic force or payload mass",
        ),
        "hydraulic_pressure": Availability(
            False,
            "lift/tilt chamber pressure",
            "Pa",
            "unavailable",
            "No hydraulic chambers/areas/flow model exists; rotary drive torques cannot supply this signal",
        ),
        "piston_stroke": Availability(
            False,
            "lift/tilt cylinder stroke",
            "m",
            "unavailable",
            "Rendered cylinders have unverified mounts and no retracted stroke reference",
        ),
        "payload_mass": Availability(
            False,
            "bucket payload mass",
            "kg",
            "unavailable",
            "Rigid pile does not fill bucket; contact force is not a weighing measurement",
        ),
    }
    ids = np.asarray(geom_ids)
    pile_geoms = [
        i
        for i in range(model.ngeom)
        if "Pile" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "")
    ]
    pile_hit_count = int(np.sum(masks & np.isin(ids, pile_geoms)))
    checks = {
        "finite_odometry": bool(np.isfinite(odom).all()),
        "scan_contract": True,
        "nonempty_observed_surface_map": bool(np.any(grid.count)),
        "pile_seen_in_privileged_ray_labels": pile_hit_count > 0,
        "missing_hydraulics_not_fabricated": not channels[
            "hydraulic_pressure"
        ].available,
        "missing_payload_not_fabricated": not channels["payload_mass"].available,
    }
    result = {
        "checks": checks,
        "mode": "offline sensor synthesis; no dynamics advanced",
        "source_model_sha256": hashlib.sha256(xml.read_bytes()).hexdigest(),
        "encoder_hz": config["encoder_hz"],
        "lidar_hz": config["lidar_hz"],
        "scan_count": len(s_idx),
        "rays_per_scan": len(dirs),
        "valid_returns": int(masks.sum()),
        "privileged_pile_hit_count": pile_hit_count,
        "observed_grid_cells": int(np.sum(grid.count > 0)),
        "odometry_position_error_rms_m": float(np.sqrt(np.mean(drift**2))),
        "odometry_position_error_max_m": float(drift.max()),
        "odometry_final_position_error_m": float(drift[-1]),
        "channels": {k: asdict(vv) for k, vv in channels.items()},
        "config": config,
        "map_semantics": "Maximum observed surface height in odom frame; includes vehicle and other surfaces, not segmented pile or unseen-volume truth",
        "physical_sensor_accuracy_validated": False,
        "production_training_sensor_backend": False,
    }
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output / "sensors.npz",
        encoder_time_s=t,
        joint_names=names,
        joint_positions_rad=positions[e_idx],
        wheel_angles_rad=wheels,
        articulation_rad=art,
        odometry_rear_xy_yaw=odom,
        imu_specific_force_body_m_s2=specific_force,
        imu_angular_rate_body_rad_s=gyro,
        imu_valid=imu_valid,
        bucket_contact_force_world_N=contact[e_idx]
        .reshape(len(e_idx), -1, 3)
        .sum(axis=1),
        scan_capture_time_s=times[s_idx],
        scan_arrival_time_s=times[s_idx] + spec["latency_s"],
        ranges_m=scans,
        valid_returns=masks,
        directions_lidar=dirs,
        points_odom_m=mapped,
        height_grid_m=grid.height,
        height_grid_count=grid.count,
        source_scan_indices=s_idx,
        source_encoder_indices=e_idx,
    )
    np.savez_compressed(
        output / "privileged_evaluation.npz",
        rear_truth_xy_yaw=truth,
        scan_origins_world_m=origins,
        scan_rotations_world=world_rot,
        ray_geom_ids=ids,
    )
    (output / "sensor_report.json").write_text(json.dumps(result, indent=2) + "\n")
    if not all(checks.values()):
        raise RuntimeError("Sensor diagnostic checks failed; inspect report")
    return result
