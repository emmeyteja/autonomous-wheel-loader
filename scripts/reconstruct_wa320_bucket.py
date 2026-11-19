"""Trace/register the brochure bucket and audit competing pin/rod interpretations."""

import argparse
import copy
import json
from pathlib import Path

import numpy as np
import yaml

from wheel_loader.vehicle.bucket_geometry import reconstruct_bucket


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-config",
        type=Path,
        default=Path("configs/geometry/wa320_7_brochure.yaml"),
    )
    parser.add_argument(
        "--bucket-config",
        type=Path,
        default=Path("configs/geometry/wa320_7_bucket.yaml"),
    )
    parser.add_argument(
        "--output", type=Path, default=Path("outputs/wa320/bucket_geometry")
    )
    parser.add_argument("--source-image", type=Path)
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    base, bucket = [
        yaml.safe_load(p.read_text()) for p in (args.base_config, args.bucket_config)
    ]
    model, poses, local, profile, report = reconstruct_bucket(base, bucket)
    rng = np.random.default_rng(bucket["uncertainty"]["seed"])
    trials = []
    for _ in range(bucket["uncertainty"]["monte_carlo_samples"]):
        sample = copy.deepcopy(bucket)
        for pose in sample["poses"].values():
            for key in ("anchor_px", "lip_px", "mouth_top_px"):
                bound = bucket["uncertainty"][
                    "pin_pick_bound_px"
                    if key == "anchor_px"
                    else "feature_pick_bound_px"
                ]
                pose[key] = (
                    np.asarray(pose[key]) + rng.uniform(-bound, bound, 2)
                ).tolist()
            pose["floor_angle_deg"] += rng.uniform(
                -bucket["uncertainty"]["angle_bound_deg"],
                bucket["uncertainty"]["angle_bound_deg"],
            )
        for name in sample["anchor_alternatives_raised_px"]:
            bound = bucket["uncertainty"]["pin_pick_bound_px"]
            sample["anchor_alternatives_raised_px"][name] = (
                sample["poses"]["raised_rollback"]["anchor_px"]
                if name == "hypothesis_1"
                else (
                    np.array(sample["anchor_alternatives_raised_px"][name])
                    + rng.uniform(-bound, bound, 2)
                ).tolist()
            )
        _, _, _, _, r = reconstruct_bucket(base, sample)
        trials.append(r["preferred_anchor_interpretation"] == "hypothesis_1")
    report["anchor_sensitivity"] = {
        "hypothesis_1_preferred_fraction": float(np.mean(trials)),
        "samples": len(trials),
        "interpretation": "Sensitivity under assumed pixel/angle perturbations only; not a probability of correct pin identity. Base/boom picks and source illustration errors are excluded.",
    }
    # Record a prescribed ground->raise/curl->dump->return cycle. No rod is driven or stretched.
    times = np.arange(401) / 25
    knots_t = [0, 2, 6, 8, 10, 12, 16]
    knots_lift = [0, 0, 1, 1, 1, 1, 0]
    knots_bucket = [0, 0, 66, 66, -45, -45, 0]
    fractions, angles = [], []
    for t in times:
        segment = min(np.searchsorted(knots_t, t, side="right") - 1, len(knots_t) - 2)
        a, b = knots_t[segment : segment + 2]
        u = (t - a) / (b - a)
        u = u * u * (3 - 2 * u)
        fractions.append((1 - u) * knots_lift[segment] + u * knots_lift[segment + 1])
        angles.append(
            np.deg2rad((1 - u) * knots_bucket[segment] + u * knots_bucket[segment + 1])
        )
    points = np.array(
        [[model.pose(float(f))[k] for k in ("B", "P", "C", "E")] for f in fractions]
    )
    from wheel_loader.vehicle.brochure_geometry import rotation

    world_profiles = np.array(
        [p[1] + profile @ rotation(a).T for p, a in zip(points, angles)]
    )
    world_anchors = np.array(
        [p[1] + rotation(a) @ local["anchor"] for p, a in zip(points, angles)]
    )
    distances = np.linalg.norm(
        world_profiles[:, :, None, :] - world_profiles[:, None, :, :], axis=-1
    )
    rigidity_error = float(np.max(np.abs(distances - distances[0])))
    report["max_profile_rigidity_error_m"] = rigidity_error
    report["recorded_ground_lip_height_m"] = float(world_profiles[0, 0, 1])
    report["recording_checks"] = {
        "finite": bool(
            np.isfinite(points).all()
            and np.isfinite(world_profiles).all()
            and np.isfinite(world_anchors).all()
        ),
        "returned_to_start": bool(
            np.allclose(points[0], points[-1]) and np.isclose(angles[0], angles[-1])
        ),
        "bucket_feature_rigidity": rigidity_error < 1e-10,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output / "bucket_kinematics.npz",
        time_s=times,
        points_XZ_m=points,
        bucket_angles_rad=angles,
        profile_local_m=profile,
        anchor_local_m=local["anchor"],
        bucket_profile_XZ_m=world_profiles,
        anchor_world_XZ_m=world_anchors,
    )
    (args.output / "bucket_reconstruction.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    from wheel_loader.evaluation.bucket_artifacts import render_bucket_artifacts

    render_bucket_artifacts(
        args.output,
        base,
        bucket,
        poses,
        report,
        source_image=args.source_image,
        video=args.video,
    )
    print(json.dumps(report, indent=2))
    if not all(report["recording_checks"].values()):
        raise SystemExit("Invalid kinematic recording")


if __name__ == "__main__":
    main()
