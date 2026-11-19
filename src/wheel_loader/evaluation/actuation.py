"""Check recorded actuator commands/forces/lengths without a simulator."""

import numpy as np


def check_actuation(p, recording):
    names = ("steering", "lift", "bucket")
    lo = np.array([p["cylinders"][n]["pin_distance_min"] for n in names])
    hi = np.array([p["cylinders"][n]["pin_distance_max"] for n in names])
    cap = np.array([p["actuators"][n]["fmax"] for n in names])
    actual = np.asarray(recording["actual_length_m"])
    reference = np.asarray(recording["reference_length_m"])
    force = np.asarray(recording["force_N"])
    u = np.asarray(recording["normalized"])
    if (
        actual.ndim != 2
        or actual.shape[1] != 3
        or reference.shape != actual.shape
        or force.shape != actual.shape
        or u.shape != (len(actual), 4)
        or len(actual) == 0
    ):
        raise ValueError("Invalid actuator recording dimensions")
    finite = all(np.isfinite(x).all() for x in (actual, reference, force, u))
    overrun = np.maximum(np.maximum(lo - actual, actual - hi), 0)
    return {
        "checks": {
            "finite_actuation": finite,
            "normalized_command_limits": bool(np.all(abs(u) <= 1)),
            "cylinder_reference_limits": bool(
                np.all(reference >= lo) and np.all(reference <= hi)
            ),
            "cylinder_force_limits": bool(np.all(abs(force) <= cap)),
            "physical_cylinder_lengths": bool(np.all(overrun < 0.001)),
        },
        "actual_length_min_m": actual.min(axis=0).tolist(),
        "actual_length_max_m": actual.max(axis=0).tolist(),
        "maximum_length_overrun_m": overrun.max(axis=0).tolist(),
        "peak_force_N": abs(force).max(axis=0).tolist(),
        "force_limit_N": cap.tolist(),
        "physical_sensor_accuracy_validated": False,
    }


def check_workspace(p, recording):
    """Measured workspace of recorded joint states, not commanded target poses."""
    from wheel_loader.control.v_cycle import PHASES
    from wheel_loader.vehicle.kinematics import rotate, solve_linkage

    names = recording["joint_names"].tolist()
    q = recording["positions"]
    roots = recording["root_poses_xyzw"]
    phases = recording["phase_ids"]
    neutral = solve_linkage(p).bucket_angle
    # Front-relative X/Z is sufficient for level-ground workspace; actual root Z
    # is used and measured roll/pitch remains separately bounded in dynamics tests.
    if len(q) == 0 or not all(np.isfinite(x).all() for x in (q, roots)):
        return {"checks": {"finite_recorded_workspace": False}}
    # Include each phase endpoint so a sparse sampling stride cannot miss it.
    indices = np.unique(
        np.r_[
            np.arange(0, len(q), 12),
            np.flatnonzero(phases[1:] != phases[:-1]),
            len(q) - 1,
        ]
    )
    rows = []
    for i in indices:
        elevation = p["boom"]["initial_elevation"] + q[i, names.index("boom_joint")]
        hinge = np.asarray(p["boom"]["pivot"]) + rotate(
            elevation, [p["boom"]["length"], 0]
        )
        beta = neutral + q[i, names.index("bucket_joint")]
        theta = elevation + beta
        lip = hinge + rotate(theta, p["bucket"]["lip"])
        top = hinge + rotate(theta, p["bucket"]["back_top"])
        rows.append(
            [
                phases[i],
                hinge[1] + roots[i, 2],
                top[1] + roots[i, 2],
                lip[1] + roots[i, 2],
                lip[0] - p["vehicle"]["wheelbase"] / 2 - p["vehicle"]["wheel_radius"],
            ]
        )
    a = np.asarray(rows)
    dump = a[a[:, 0] == PHASES.index("dump")]
    if not len(dump):
        return {"checks": {"recorded_dump_workspace": False}}
    last = dump[-1]
    rollback = a[a[:, 0] == PHASES.index("raise_dump")]
    if not len(rollback):
        return {"checks": {"recorded_raised_rollback_workspace": False}}
    measured = {
        "maximum_hinge_height_m": float(a[:, 1].max()),
        "maximum_bucket_height_m": float(a[:, 2].max()),
        "raised_rollback_bucket_height_m": float(rollback[-1, 2]),
        "final_dump_clearance_m": float(last[3]),
        "final_dump_reach_m": float(last[4]),
    }
    return {
        "measured": measured,
        "checks": {
            "recorded_hinge_height": abs(
                measured["maximum_hinge_height_m"] - p["vehicle"]["hinge_pin_height"]
            )
            < 0.03,
            "recorded_bucket_height": abs(
                measured["raised_rollback_bucket_height_m"]
                - p["vehicle"]["bucket_max_height"]
            )
            < 0.08,
            "recorded_dump_clearance": bool(
                abs(last[3] - p["vehicle"]["dump_clearance"]) < 0.08
            ),
            "recorded_dump_reach": bool(
                abs(last[4] - p["vehicle"]["dump_reach"]) < 0.10
            ),
        },
        "screen_tolerance_m": {
            "hinge": 0.03,
            "bucket_height": 0.08,
            "dump_clearance": 0.08,
            "dump_reach": 0.10,
        },
        "independent_geometry_accuracy": False,
    }
