"""Export v1 mass/COM/inertia, static support and workspace diagnostics."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from wheel_loader.evaluation.foundation import check_geometry
from wheel_loader.vehicle.kinematics import rotate, solve_linkage
from wheel_loader.vehicle.mass_properties import mass_report
from wheel_loader.vehicle.parameters import load_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/vehicle/wa320.yaml")
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/wa320/reconstructed_v1/mechanics.json"),
    )
    parser.add_argument(
        "--cycle",
        type=Path,
        default=Path("outputs/wa320/reconstructed_v1/release_nominal"),
    )
    parser.add_argument(
        "--dynamics",
        type=Path,
        default=Path("outputs/wa320/reconstructed_v1/release_dynamics"),
    )
    args = parser.parse_args()
    p = load_parameters(args.config)
    digest = hashlib.sha256(json.dumps(p, sort_keys=True).encode()).hexdigest()
    report = mass_report(p)
    report["parameter_sha256"] = digest
    report["linkage"] = check_geometry(p)
    rows = []
    for name, (a, t) in p["maneuver"].items():
        pose = solve_linkage(p, a, t)
        rod = np.asarray(p["geometry"]["lift_cylinder_rod_pin"])
        base = np.asarray(p["geometry"]["lift_cylinder_base"])
        pin = np.array([pose.boom_pin[0], 0, pose.boom_pin[1]]) + np.array(
            [
                *rotate(pose.boom_elevation, [rod[0], rod[2]])[:1],
                rod[1],
                rotate(pose.boom_elevation, [rod[0], rod[2]])[1],
            ]
        )
        length = float(np.linalg.norm(pin - base))
        rows.append(
            {
                "pose": name,
                "lift_pin_length_m": length,
                "bucket_pin_length_m": pose.cylinder_length,
                "cylinder_lengths_in_range": bool(
                    p["cylinders"]["lift"]["pin_distance_min"] - 1e-6
                    <= length
                    <= p["cylinders"]["lift"]["pin_distance_max"] + 1e-6
                    and p["cylinders"]["bucket"]["pin_distance_min"]
                    <= pose.cylinder_length
                    <= p["cylinders"]["bucket"]["pin_distance_max"]
                ),
                "hinge_height_m": float(
                    pose.bucket_pin[1] + p["vehicle"]["articulation_height"]
                ),
                "lip_height_m": float(
                    pose.lip[1] + p["vehicle"]["articulation_height"]
                ),
            }
        )
    report["maneuver_workspace"] = rows
    report["checks"] = {
        "mass_budget": abs(report["vehicle_mass_kg"] - 15175) <= 1,
        "static_axle_sanity": 30 < report["front_percentage"] < 60
        and report["static_support_positive"],
        "loop_algebraic_closure": report["linkage"]["max_loop_residual_m"] < 1e-10,
        "maneuver_cylinder_envelopes": all(
            r["cylinder_lengths_in_range"] for r in rows
        ),
    }
    cycle_dir = args.cycle
    dynamics_file = args.dynamics / "suite.json"
    cycle_file = cycle_dir / "validation.json"
    if cycle_file.exists() and (cycle_dir / "actuation.npz").exists():
        from wheel_loader.evaluation.actuation import check_actuation, check_workspace

        report["newton_cycle"] = json.loads(cycle_file.read_text())
        with np.load(cycle_dir / "actuation.npz") as n:
            report["actuation"] = check_actuation(p, n)
        report["checks"]["cycle_parameters_match"] = (
            report["newton_cycle"].get("parameter_sha256") == digest
        )
        with np.load(cycle_dir / "motion.npz") as n:
            report["workspace"] = check_workspace(p, n)
        report["checks"].update(report["workspace"]["checks"])
        report["checks"]["newton_v_cycle"] = all(
            report["newton_cycle"]["checks"].values()
        )
        report["checks"]["recorded_actuator_limits"] = all(
            report["actuation"]["checks"].values()
        )
    else:
        report["checks"]["newton_v_cycle"] = False
    if dynamics_file.exists():
        report["dynamic_sanity"] = json.loads(dynamics_file.read_text())
        report["checks"]["seven_dynamic_cases"] = len(
            report["dynamic_sanity"]
        ) == 7 and all(
            all(r["checks"].values()) and r.get("parameter_sha256") == digest
            for r in report["dynamic_sanity"].values()
        )
    else:
        report["checks"]["seven_dynamic_cases"] = False
    report["v1_complete"] = bool(all(report["checks"].values()))
    report["physical_accuracy_established"] = False
    report["remaining_unknowns"] = [
        "true pin/COM/inertia errors and installed equipment masses",
        "calibrated cylinder lag, speed, force, friction and power sharing",
        "3.0 m3 bucket mass and profile measurements",
        "granular loading and payload estimation, excluded from v1",
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            {k: v for k, v in report.items() if k != "body_properties"}, indent=2
        )
    )


if __name__ == "__main__":
    main()
