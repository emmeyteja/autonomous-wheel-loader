"""Reconstruct the identifiable WA320-7 boom geometry from explicit drawing picks."""

import argparse
import copy
import json
from pathlib import Path

import numpy as np
import yaml

from wheel_loader.vehicle.brochure_geometry import reconstruct
from wheel_loader.vehicle.kinematics import solve_linkage
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/geometry/wa320_7_brochure.yaml")
    )
    parser.add_argument(
        "--vehicle", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("outputs/wa320/brochure_geometry")
    )
    parser.add_argument(
        "--source-image",
        type=Path,
        help="Optional local 4240x6000 brochure raster for annotated crop",
    )
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    model, observed, report = reconstruct(config)
    rng = np.random.default_rng(config["uncertainty"]["seed"])
    sensitivities = []
    for _ in range(config["uncertainty"]["monte_carlo_samples"]):
        sample = copy.deepcopy(config)
        for name in ("lowered", "raised"):
            for key in ("bucket_pin_px", "crank_pivot_px", "tilt_pin_px"):
                bound = config["uncertainty"]["visible_pin_pick_bound_px"]
                sample["picks"][name][key] = (
                    np.array(config["picks"][name][key]) + rng.uniform(-bound, bound, 2)
                ).tolist()
        for key in ("front_axle_px", "rear_axle_px"):
            sample["calibration"][key] = (
                np.array(config["calibration"][key]) + rng.uniform(-1, 1, 2)
            ).tolist()
        sample["calibration"]["ground_y_px"] += rng.uniform(-1, 1)
        fitted, _, _ = reconstruct(sample)
        sensitivities.append(
            [*fitted.base, np.linalg.norm(fitted.bucket_local), fitted.crank_local[1]]
        )
    report["pick_sensitivity"] = {
        "interpretation": "5th/95th percentiles under assumed independent uniform pixel perturbations; not confidence intervals or physical accuracy bounds",
        "columns": ["base_X_m", "base_Z_m", "boom_length_m", "crank_offset_Z_m"],
        "p05": np.quantile(sensitivities, 0.05, axis=0).tolist(),
        "p95": np.quantile(sensitivities, 0.95, axis=0).tolist(),
        "scope": "Visible P/C/E picks and drawing scale/ground only; excludes pin misidentification, perspective, hidden geometry and illustration errors",
    }
    vehicle = load_parameters(args.vehicle)
    old = solve_linkage(vehicle)
    shift = np.array(
        [
            -vehicle["vehicle"]["wheelbase"] / 2,
            vehicle["vehicle"]["articulation_height"],
        ]
    )
    report["active_model_comparison"] = {
        "assumption": "Equal hinge-to-axle distances retained only to express active coordinates in drawing frame",
        "nominal_boom_base_error_m": float(
            np.linalg.norm(old.boom_pin + shift - model.base)
        ),
        "nominal_bucket_pin_error_m": float(
            np.linalg.norm(old.bucket_pin + shift - observed[0]["P"])
        ),
        "active_boom_length_m": vehicle["boom"]["length"],
        "active_crank_offset_Z_m": 0.0,
        "active_crank_included_angle_deg": 180.0,
    }
    report["checks"] = {
        "fit_pin_residual_under_30mm": max(
            v for pose in report["pin_residuals_m"].values() for v in pose.values()
        )
        < 0.03,
        "independent_visible_base_under_60mm": report[
            "independent_visible_base_error_m"
        ]
        < 0.06,
        "published_raised_pin_height_under_40mm": abs(
            report["raised_pin_height_error_m"]
        )
        < 0.04,
        "wheel_height_scale_under_20mm": abs(report["nominal_wheel_height_error_m"])
        < 0.02,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "reconstruction.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    # Record the reconstructed states; the renderer consumes this recording.
    times = np.arange(301) / 25
    fraction = 0.5 - 0.5 * np.cos(2 * np.pi * times / times[-1])
    labels = ["B", "P", "C", "E", "D", "T"]
    points = np.array([[model.pose(float(f))[k] for k in labels] for f in fraction])
    np.savez_compressed(
        args.output / "kinematics.npz",
        time_s=times,
        points_XZ_m=points,
        labels=labels,
        fraction=fraction,
    )
    from wheel_loader.evaluation.brochure_artifacts import render_artifacts

    render_artifacts(
        args.output,
        config,
        model,
        observed,
        report,
        source_image=args.source_image,
        video=args.video,
    )
    print(json.dumps(report, indent=2))
    if not all(report["checks"].values()):
        raise SystemExit("Drawing reconstruction checks failed")


if __name__ == "__main__":
    main()
