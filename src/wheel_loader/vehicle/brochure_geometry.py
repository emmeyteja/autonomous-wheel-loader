"""Drawing-derived planar subassembly; no dynamics or complete PZ identification."""

from dataclasses import dataclass

import numpy as np


def rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s], [s, c]])


def fit_rigid_motion(lower, raised):
    """Least-squares proper rigid transform and its fixed planar rotation centre."""
    lower, raised = np.asarray(lower, float), np.asarray(raised, float)
    if (
        lower.shape != raised.shape
        or lower.ndim != 2
        or lower.shape[1] != 2
        or len(lower) < 2
        or not np.isfinite(lower).all()
        or not np.isfinite(raised).all()
        or np.linalg.norm(lower - lower.mean(0)) < 1e-8
        or np.linalg.norm(raised - raised.mean(0)) < 1e-8
    ):
        raise ValueError("Need finite corresponding separated planar points")
    a, b = lower.mean(0), raised.mean(0)
    u, _, vt = np.linalg.svd((lower - a).T @ (raised - b))
    correction = np.diag([1, np.linalg.det(vt.T @ u.T)])
    r = vt.T @ correction @ u.T
    if np.linalg.cond(np.eye(2) - r) > 1e8 or np.linalg.norm(np.eye(2) - r) < 1e-8:
        raise ValueError("Rotation centre is unidentifiable for near-zero rotation")
    offset = b - r @ a
    centre = np.linalg.solve(np.eye(2) - r, offset)
    return centre, float(np.arctan2(r[1, 0], r[0, 0])), r


@dataclass(frozen=True)
class BrochureBoom:
    base: np.ndarray
    bucket_local: np.ndarray
    crank_local: np.ndarray
    upper_length: float
    rod_local: np.ndarray
    tilt_base: np.ndarray
    lift_angles: tuple
    crank_angles: tuple

    def pose(self, fraction):
        """Prescribed interpolation, not a hydraulic law or observed trajectory."""
        if not np.isfinite(fraction) or not 0 <= fraction <= 1:
            raise ValueError("Interpolation fraction must be in [0, 1]")
        q = float(np.interp(fraction, [0, 1], self.lift_angles))
        phi = float(np.interp(fraction, [0, 1], self.crank_angles))
        p = self.base + rotation(q) @ self.bucket_local
        c = self.base + rotation(q) @ self.crank_local
        e = c + rotation(q + phi) @ [self.upper_length, 0]
        d = c + rotation(q + phi) @ self.rod_local
        return {"B": self.base, "P": p, "C": c, "E": e, "D": d, "T": self.tilt_base}


def reconstruct(config):
    cal, picks = config["calibration"], config["picks"]
    front, rear = np.asarray(cal["front_axle_px"]), np.asarray(cal["rear_axle_px"])
    span = rear[0] - front[0]
    if span <= 0 or cal["wheelbase_m"] <= 0:
        raise ValueError("Invalid drawing scale")
    scale = cal["wheelbase_m"] / span
    origin = np.array([front[0], cal["ground_y_px"]])

    def world(pixel):
        return (origin - np.asarray(pixel)) * scale

    observed = [
        {
            key: world(picks[name][field])
            for key, field in (
                ("P", "bucket_pin_px"),
                ("C", "crank_pivot_px"),
                ("E", "tilt_pin_px"),
            )
        }
        for name in ("lowered", "raised")
    ]
    a, b = [np.array([pose["P"], pose["C"]]) for pose in observed]
    base, sweep, _r = fit_rigid_motion(a, b)
    q0 = np.arctan2(*(a[0] - base)[::-1])
    q1 = q0 + sweep
    locals_ = [rotation(-q) @ (points - base).T for q, points in ((q0, a), (q1, b))]
    local = (locals_[0] + locals_[1]) / 2
    upper = [o["E"] - o["C"] for o in observed]
    upper_length = float(np.mean([np.linalg.norm(v) for v in upper]))
    phis = tuple(float(np.arctan2(*v[::-1]) - q) for v, q in zip(upper, (q0, q1)))
    rod = rotation(-(q1 + phis[1])) @ (world(picks["raised"]["rod_pin_px"]) - b[1])
    model = BrochureBoom(
        base,
        local[:, 0],
        local[:, 1],
        upper_length,
        rod,
        world(picks["tilt_base_px"]),
        (float(q0), float(q1)),
        phis,
    )
    residuals = {}
    for i, pose in enumerate(observed):
        fit = model.pose(i)
        residuals[("lowered", "raised")[i]] = {
            k: float(np.linalg.norm(fit[k] - pose[k])) for k in pose
        }
    radius, pub = config["published"]["wheel_radius_m"], config["published"]
    centre_radius = (
        cal["wheelbase_m"] / 2 / np.tan(np.deg2rad(pub["articulation_deg"]) / 2)
    )
    tire_width = pub["outside_tires_width_m"] - pub["track_m"]
    report = {
        "metres_per_pixel": scale,
        "frame": config["frame"],
        "boom_base_m": base.tolist(),
        "boom_length_m": float(np.linalg.norm(model.bucket_local)),
        "crank_pivot_in_boom_m": model.crank_local.tolist(),
        "crank_tilt_pin_radius_m": upper_length,
        "crank_rod_pin_local_m": rod.tolist(),
        "crank_included_angle_deg": float(np.rad2deg(np.arctan2(rod[1], rod[0]))),
        "lift_angles_deg": np.rad2deg(model.lift_angles).tolist(),
        "pin_residuals_m": residuals,
        "independent_visible_base_error_m": float(
            np.linalg.norm(base - world(picks["boom_base_px"]))
        ),
        "raised_pin_height_error_m": float(
            model.pose(1)["P"][1] - pub["raised_bucket_pin_height_m"]
        ),
        "nominal_wheel_height_error_m": float(world(cal["front_axle_px"])[1] - radius),
        "turning_reference": {
            "assumptions": "Equal hinge-to-axle offsets, no slip, symmetric nominal tire widths",
            "outer_wheel_centre_radius_m": float(centre_radius + pub["track_m"] / 2),
            "inner_tire_envelope_radius_m": float(
                centre_radius - pub["track_m"] / 2 - tire_width / 2
            ),
            "derived_nominal_tire_width_m": tire_width,
        },
        "rod_length_hypotheses_m": [
            float(np.linalg.norm(world(f) - world(picks["raised"]["rod_pin_px"])))
            for f in picks["raised"]["bucket_anchor_hypotheses_px"]
        ],
        "full_linkage_identified": False,
        "dynamics_validated": False,
        "production_geometry_promoted": False,
    }
    return model, observed, report
