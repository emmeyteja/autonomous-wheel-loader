"""Save the reconstructed loader's deterministic linkage signature and plot."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from wheel_loader.evaluation.mechanical_signature import linkage_signature
from wheel_loader.vehicle.parameters import load_parameters

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/wa320/mechanical_correction/signature"),
    )
    args = parser.parse_args()
    p = load_parameters(args.config)
    rows = linkage_signature(p)
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "signature.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    x = np.rad2deg([r["boom_rad"] for r in rows])
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), layout="constrained")
    axes[0].plot(x, np.rad2deg([r["bucket_rad"] for r in rows]))
    axes[0].set_ylabel("Bucket relative to boom [deg]")
    for key in ("tip_x_m", "tip_z_m"):
        axes[1].plot(x, [r[key] for r in rows], label=key)
    axes[1].set_ylabel("Bucket tip [m]")
    axes[1].legend()
    for name in ("lift_left", "bucket", "steering_left"):
        axes[2].plot(x, [r[name + "_extension_m"] for r in rows], label=name)
    axes[2].set_ylabel("Cylinder extension from manual minimum [m]")
    axes[2].set_xlabel("Boom displacement [deg]")
    axes[2].legend()
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.suptitle(
        "WA320 reconstructed v1 — estimated geometry\nDeterministic lower-to-raised-rollback signature; not independent machine validation"
    )
    fig.savefig(args.output / "signature.png", dpi=160)
    print(
        json.dumps(
            {
                "poses": len(rows),
                "maximum_zbar_gap_m": max(r["zbar_gap_m"] for r in rows),
                "maximum_cylinder_gap_m": max(
                    r["maximum_cylinder_pin_gap_m"] for r in rows
                ),
            },
            indent=2,
        )
    )
