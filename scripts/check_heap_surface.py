"""Check a leveled reference surface against an independently known cone angle."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from wheel_loader.evaluation.column import surface_profiles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--delimiter", choices=["comma", "whitespace"], default="whitespace"
    )
    parser.add_argument(
        "--base-z",
        type=float,
        required=True,
        help="Declared base datum in source coordinate units",
    )
    parser.add_argument(
        "--bin-width",
        type=float,
        required=True,
        help="In the same source coordinate units",
    )
    parser.add_argument("--expected-angle-deg", type=float, required=True)
    parser.add_argument("--tolerance-deg", type=float, default=0.5)
    parser.add_argument("--source-doi", required=True)
    args = parser.parse_args()
    if not (
        0 < args.expected_angle_deg < 90
        and 0 < args.tolerance_deg < 90
        and np.isfinite(args.base_z)
    ):
        parser.error("Invalid angle, tolerance or base datum")
    points = np.loadtxt(
        args.input,
        delimiter="," if args.delimiter == "comma" else None,
        usecols=(0, 1, 2),
    )
    points[:, 2] -= args.base_z
    fits = surface_profiles(points, args.bin_width)
    passed = all(
        fit is not None
        and abs(fit["angle_deg"] - args.expected_angle_deg) <= args.tolerance_deg
        for fit in fits.values()
    )
    with args.input.open("rb") as stream:
        source_hash = hashlib.file_digest(stream, "sha256").hexdigest()
    report = {
        "source_doi": args.source_doi,
        "input_sha256": source_hash,
        "point_count": len(points),
        "base_z_source_units": args.base_z,
        "bin_width_source_units": args.bin_width,
        "expected_angle_deg": args.expected_angle_deg,
        "tolerance_deg": args.tolerance_deg,
        "surface_sector_fits": fits,
        "geometry_measurement_check_passed": passed,
        "physical_material_validation_passed": False,
        "assumptions": [
            "Input is leveled with Z normal to base",
            "No automatic ground removal",
            "First three columns are XYZ with equal axis scales",
            "Mean XY sampling position used as profile center",
            "Angle-only check; no absolute length-unit or material calibration claim",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit("Surface measurement check failed")


if __name__ == "__main__":
    main()
