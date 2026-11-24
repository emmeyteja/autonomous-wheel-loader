"""Opt-in geometry feedback test; frozen benchmark and original controller stay intact."""

import argparse
import hashlib
import json
import runpy
from pathlib import Path

import numpy as np
import yaml

from wheel_loader.environment.simulation.backend_benchmark import (
    load_bundle,
    run_replay,
)
from wheel_loader.environment.simulation.spatial_scoop import SpatialScoopFeedback
from wheel_loader.vehicle.parameters import load_parameters

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--backend", choices=["native", "mjwarp"], default="native")
    p.add_argument("--unloaded", action="store_true")
    p.add_argument("--seconds", type=float, default=80)
    p.add_argument(
        "--config", type=Path, default=Path("configs/experiments/spatial_scoop.yaml")
    )
    a = p.parse_args()
    folder = a.output
    if folder.exists() and any(folder.iterdir()):
        p.error("Output directory must be empty; preserve prior trials")
    if not np.isfinite(a.seconds) or a.seconds <= 0:
        p.error("Duration must be finite and positive")
    bundle = folder / "bundle"
    runpy.run_path("scripts/prepare_mechanics_benchmark.py")["prepare"](
        Path("outputs/wa320/backend_benchmark/bundle"),
        bundle,
        "unloaded_ground" if a.unloaded else "pile_reference",
        33.30833333333333,
        solver="Newton",
    )
    model, tape, manifest = load_bundle(bundle)
    cfg = yaml.safe_load(a.config.read_text())
    cfg["unloaded"] = a.unloaded
    steps = round(a.seconds / manifest["control_dt_s"])
    tape["effort"] = np.zeros((steps, model.nu))
    tape["phase"] = np.full(steps, "geometry_feedback")
    np.savez_compressed(bundle / "command_tape.npz", **tape)
    manifest.update(
        control_steps=steps,
        duration_s=a.seconds,
        actuation="Live bounded physical cylinder feedback; zero tape placeholders unused",
        controller_configuration=cfg,
    )
    manifest["sha256"]["command_tape.npz"] = hashlib.sha256(
        (bundle / "command_tape.npz").read_bytes()
    ).hexdigest()
    (bundle / "benchmark.json").write_text(json.dumps(manifest, indent=2) + "\n")
    parameters = load_parameters("configs/vehicle/wa320.yaml")
    controller = SpatialScoopFeedback(model, parameters, cfg)
    report = run_replay(
        bundle,
        folder,
        a.backend,
        parameters,
        audit=True,
        feedback=controller,
        direct_contact_forces=True,
    )
    np.savez_compressed(
        folder / "controller.npz",
        values=np.array(controller.log),
        columns=[
            "time_s",
            "lip_x_m",
            "lip_z_m",
            "cutting_pitch_rad",
            "ground_clearance_m",
            "penetration_m",
            "payload_kg",
            "active_bucket_material_contact",
            "height_target_m",
            "pitch_target_rad",
            "speed_target_m_s",
            "lift_velocity_command_m_s",
            "curl_velocity_command_m_s",
            "bucket_reaction_N",
        ],
    )
    report.update(
        controller_done=controller.controller.done,
        controller_failed=controller.controller.failed,
        controller_reason=controller.controller.reason,
        events=controller.controller.events,
        controller_configuration=cfg,
        passed=bool(controller.controller.done and not controller.controller.failed),
        scope="Unloaded motion test"
        if a.unloaded
        else "Experimental physical scoop; uncalibrated coarse spheres",
        selected_entry_plane_x_m=controller.entry_x,
    )
    (folder / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("Spatial controller acceptance failed")
