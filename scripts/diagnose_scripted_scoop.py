"""Inspect recorded scoop classifier/attack evidence without advancing dynamics."""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from wheel_loader.evaluation.material_accounting import (
    bucket_payload_mask,
    inside_bucket,
)
from wheel_loader.vehicle.parameters import load_parameters


def diagnose(folder, parameters):
    if not (folder / "report.json").exists():
        raise ValueError(
            "A final run report is required; unfinished runs may contain stale replay files"
        )
    model = mujoco.MjModel.from_xml_path(str(folder / "model.xml"))
    data = mujoco.MjData(model)
    bucket = next(
        b
        for b in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "").endswith(
            "_bucket"
        )
    )
    profiles = {"plate_clearance_classifier": [], "zero_clearance_centroid": []}
    with np.load(folder / "recording.npz") as n:
        for q in n["qpos"]:
            data.qpos[:] = q
            mujoco.mj_kinematics(model, data)
            local = (data.xpos[n["grain_body_ids"]] - data.xpos[bucket]) @ data.xmat[
                bucket
            ].reshape(3, 3)
            masks = {
                "plate_clearance_classifier": bucket_payload_mask(local, parameters),
                "zero_clearance_centroid": inside_bucket(
                    local,
                    parameters["bucket"]["profile"],
                    parameters["vehicle"]["bucket_width"] / 2,
                    np.zeros(len(local)),
                ),
            }
            for key, mask in masks.items():
                profiles[key].append(
                    {
                        "count": int(mask.sum()),
                        "mass_kg": float(n["masses_kg"][mask].sum()),
                    }
                )
        active = np.isin(n["phase"], ["penetrate", "curl_advance"])
        lip = (
            n["bucket_lip_world_m"][active] if "bucket_lip_world_m" in n.files else None
        )
        result = {
            "method": "recorded qpos replay with forward kinematics only; no dynamics steps or state correction",
            "classifiers": {
                key: {
                    "peak_count": max(r["count"] for r in values),
                    "peak_mass_kg": max(r["mass_kg"] for r in values),
                    "final": values[-1],
                }
                for key, values in profiles.items()
            },
            "measured_penetration_lip_height_range_m": [
                float(lip[:, 2].min()),
                float(lip[:, 2].max()),
            ]
            if lip is not None and len(lip)
            else None,
            "interpretation": "If both classifiers find zero entry, the plate-clearance exclusion alone does not explain a zero payload. This does not validate particle resolution or contact geometry.",
        }
    (folder / "classifier_diagnosis.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    print(
        json.dumps(
            diagnose(args.folder, load_parameters("configs/vehicle/wa320.yaml")),
            indent=2,
        )
    )
