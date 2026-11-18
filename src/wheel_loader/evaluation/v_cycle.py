"""Acceptance for the full motion rehearsal; no material-acquisition claim."""

import numpy as np

from wheel_loader.control.v_cycle import PHASES


def evaluate_v_cycle(
    p, scenario, positions, roots, phases, commands, events, completed
):
    q, roots, phases, commands = map(np.asarray, (positions, roots, phases, commands))
    names = scenario["joint_names"]
    completed_phases = [event["phase"] for event in events]
    events_by_phase = {event["phase"]: event for event in events}

    def endpoint(name, goal):
        return (
            name in events_by_phase
            and np.linalg.norm(np.asarray(events_by_phase[name]["xy_m"]) - goal) < 0.35
        )

    def joint_change(phase, joint):
        values = q[phases == PHASES.index(phase), names.index(joint)]
        return float(values[-1] - values[0]) if len(values) > 1 else 0.0

    def gear(phase, sign):
        mask = phases == PHASES.index(phase)
        indices = np.flatnonzero(mask)
        if len(indices) < 2:
            return False
        delta = roots[indices[1:], :2] - roots[indices[:-1], :2]
        x, y, z, w = roots[indices[:-1], 3:].T
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        distance = np.sum(delta * np.column_stack([np.cos(yaw), np.sin(yaw)]))
        return sign * distance > 0.25

    limits = all(
        q[:, names.index(joint)].min() >= p[section]["min_angle"] - 0.005
        and q[:, names.index(joint)].max() <= p[section]["max_angle"] + 0.005
        for joint, section in [
            ("articulation_joint", "articulation"),
            ("boom_joint", "boom"),
            ("tilt_joint", "linkage"),
            ("bucket_joint", "bucket"),
        ]
    )
    corridor = {}
    for name, start, goal in [
        ("forward_right", [0.0, 0.0], [scenario["distance"], -scenario["half_width"]]),
        ("reverse_loaded", [scenario["distance"], -scenario["half_width"]], [0.0, 0.0]),
        ("forward_left", [0.0, 0.0], [scenario["distance"], scenario["half_width"]]),
        ("reverse_home", [scenario["distance"], scenario["half_width"]], [0.0, 0.0]),
    ]:
        points = roots[phases == PHASES.index(name), :2]
        delta = np.asarray(goal) - start
        direct = float(np.linalg.norm(delta))
        cross = (
            np.abs(
                (points[:, 0] - start[0]) * delta[1]
                - (points[:, 1] - start[1]) * delta[0]
            )
            / direct
            if len(points)
            else [float("inf")]
        )
        path_length = (
            float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
            if len(points) > 1
            else float("inf")
        )
        corridor[name] = {
            "maximum_cross_track_m": float(np.max(cross)),
            "path_length_m": path_length,
            "passed": bool(np.max(cross) < 3.5 and path_length < 1.6 * direct),
        }
    reverse = phases == PHASES.index("reverse_loaded")
    final_window = min(100, len(roots))
    final_speed = float(
        np.linalg.norm(roots[-1, :2] - roots[-final_window, :2])
        / ((final_window - 1) * p["solver"]["dt"])
    )
    checks = {
        "v_path_corridor": all(record["passed"] for record in corridor.values()),
        "full_sequence": completed and completed_phases == list(PHASES),
        "forward_right": endpoint(
            "forward_right", [scenario["distance"], -scenario["half_width"]]
        )
        and gear("forward_right", 1),
        "loading_push": gear("push", 1),
        "curl_and_lift": joint_change("curl_lift", "boom_joint") > 0.3
        and joint_change("curl_lift", "bucket_joint") > 0.1,
        "reverse_with_bucket_raised": bool(
            reverse.any() and q[reverse, names.index("boom_joint")].min() > 0.3
        )
        and gear("reverse_loaded", -1),
        "return_through_v_center": endpoint("reverse_loaded", [0.0, 0.0]),
        "forward_left": endpoint(
            "forward_left", [scenario["distance"], scenario["half_width"]]
        )
        and gear("forward_left", 1),
        "dump_motion": joint_change("dump", "bucket_joint") < -0.4,
        "reverse_home": gear("reverse_home", -1),
        "return_to_origin": bool(np.linalg.norm(roots[-1, :2]) < 0.35),
        "stopped_at_end": bool(
            np.max(np.abs(commands[-100:, 0])) < 0.01 and final_speed < 0.02
        ),
        "joint_limits": bool(limits),
        "finite_states": bool(np.isfinite(q).all() and np.isfinite(roots).all()),
    }
    return {
        "checks": {name: bool(value) for name, value in checks.items()},
        "final_planar_speed_m_s": final_speed,
        "final_distance_to_origin_m": float(np.linalg.norm(roots[-1, :2])),
        "phase_events": events,
        "route_metrics": corridor,
        "material_transfer_validated": False,
    }
