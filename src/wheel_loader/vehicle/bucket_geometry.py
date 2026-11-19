"""Rigid bucket outline registration and tests of provisional rod hypotheses."""

import numpy as np

from .brochure_geometry import reconstruct, rotation


def fit_bucket_features(poses, fit_names):
    """Mean pin-relative feature coordinates at explicit floor angles (radians)."""
    if not fit_names or len(set(fit_names)) != len(fit_names):
        raise ValueError("Need distinct fitting poses")
    result = {}
    for feature in ("lip", "mouth_top", "anchor"):
        local = [
            rotation(-poses[name]["angle"]) @ (poses[name][feature] - poses[name]["P"])
            for name in fit_names
        ]
        values = np.asarray(local, float)
        if values.shape != (len(fit_names), 2) or not np.isfinite(values).all():
            raise ValueError("Invalid planar feature coordinates")
        result[feature] = values.mean(0)
    return result


def rod_lengths(model, poses, names):
    """Test a fixed-D/fixed-length-rod interpretation; does not identify a linkage."""
    return np.array(
        [
            np.linalg.norm(
                poses[name]["anchor"] - model.pose(poses[name]["fraction"])["D"]
            )
            for name in names
        ]
    )


def reconstruct_bucket(base_config, bucket_config):
    boom, _, boom_report = reconstruct(base_config)
    scale = boom_report["metres_per_pixel"]
    cal = base_config["calibration"]
    origin = np.array([cal["front_axle_px"][0], cal["ground_y_px"]])
    world = lambda pixel: (origin - np.asarray(pixel, float)) * scale
    poses = {}
    for name, values in bucket_config["poses"].items():
        fraction = values["boom_fraction"]
        ref = ("lowered", "raised")[fraction]
        poses[name] = {
            "P": world(base_config["picks"][ref]["bucket_pin_px"]),
            "fraction": fraction,
            "angle": float(np.deg2rad(values["floor_angle_deg"])),
            **{
                feature: world(values[feature + "_px"])
                for feature in ("lip", "mouth_top", "anchor")
            },
        }
    fit_names = bucket_config["fit_poses"]
    if len(fit_names) != 2:
        raise ValueError("Rod audit requires exactly two fitting poses")
    if bucket_config["held_out_pose"] in fit_names:
        raise ValueError("Held-out pose must not contribute to fitting")
    local = fit_bucket_features(poses, fit_names)
    residuals = {
        name: {
            key: float(
                np.linalg.norm(pose["P"] + rotation(pose["angle"]) @ point - pose[key])
            )
            for key, point in local.items()
        }
        for name, pose in poses.items()
    }
    profile = np.array(
        [world(p) - poses["ground"]["P"] for p in bucket_config["ground_profile_px"]]
    )
    # Keep one rigid traced outline; align its lip/mouth endpoints with the fitted features.
    profile[0], profile[-1] = local["lip"], local["mouth_top"]
    ground = poses["ground"]
    alternative_errors = {}
    for name, pixel in bucket_config["anchor_alternatives_raised_px"].items():
        raised = poses["raised_rollback"]
        candidate = rotation(-raised["angle"]) @ (world(pixel) - raised["P"])
        # Score against ground only, so no alternative participates in its own fit.
        ground_local = ground["anchor"] - ground["P"]
        alternative_errors[name] = float(np.linalg.norm(candidate - ground_local))
    lengths = rod_lengths(boom, poses, fit_names)
    # Two poses supply one independent equal-length constraint on unknown crank pin d.
    u = []
    for name in fit_names:
        pose = poses[name]
        f = pose["fraction"]
        theta = boom.lift_angles[f] + boom.crank_angles[f]
        u.append(rotation(-theta) @ (pose["anchor"] - boom.pose(f)["C"]))
    normal = 2 * (u[0] - u[1])
    rhs = float(u[0] @ u[0] - u[1] @ u[1])
    if np.linalg.norm(normal) < 1e-10:
        raise ValueError("Rod-pin constraint is degenerate")
    nearest_d = boom.rod_local + normal * (rhs - normal @ boom.rod_local) / (
        normal @ normal
    )
    # This line is not promoted to an identified point: infinitely many D/rod pairs fit.
    pub = base_config["published"]
    held = poses[bucket_config["held_out_pose"]]
    predicted_lip = held["P"] + rotation(held["angle"]) @ local["lip"]
    report = {
        "frame": base_config["frame"],
        "source": base_config["source"],
        "bucket_config": bucket_config,
        "fit_poses": fit_names,
        "held_out_pose": bucket_config["held_out_pose"],
        "local_features_m": {k: v.tolist() for k, v in local.items()},
        "profile_vertices_m": profile.tolist(),
        "pose_residuals_m": residuals,
        "raised_anchor_ground_registration_errors_m": alternative_errors,
        "preferred_anchor_interpretation": min(
            alternative_errors, key=alternative_errors.get
        ),
        "rod_hypothesis": {
            "scope": "Assumes the ground/raised crank states correspond to the registered bucket poses and one rigid connecting rod",
            "lengths_m": lengths.tolist(),
            "length_spread_m": float(np.ptp(lengths)),
            "fixed_D_single_rod_consistent": bool(np.ptp(lengths) < 0.03),
            "project_screen_m": 0.03,
            "D_constraint_normal": normal.tolist(),
            "D_constraint_rhs": rhs,
            "constraint_rank": 1,
            "unknown_D_coordinates": 2,
            "nearest_equal_length_D_local_m": nearest_d.tolist(),
            "nearest_D_change_m": float(np.linalg.norm(nearest_d - boom.rod_local)),
            "identified_D": False,
        },
        "dump_geometry_cross_check": {
            "reference": "Reach measured ahead of nominal leading tire envelope; wheel radius, not axle centre, is subtracted",
            "predicted_clearance_m": float(predicted_lip[1]),
            "published_clearance_m": pub["dump_clearance_m"],
            "clearance_error_m": float(predicted_lip[1] - pub["dump_clearance_m"]),
            "predicted_reach_m": float(predicted_lip[0] - pub["wheel_radius_m"]),
            "published_reach_m": pub["dump_reach_m"],
            "reach_error_m": float(
                predicted_lip[0] - pub["wheel_radius_m"] - pub["dump_reach_m"]
            ),
        },
        "nominal_bucket_width_m": 2.685,
        "nominal_capacity_m3": 3.0,
        "capacity_reconstructed": False,
        "collision_ready": False,
        "full_linkage_identified": False,
        "dynamics_validated": False,
        "production_geometry_promoted": False,
    }
    return boom, poses, local, profile, report
