"""Loading phase gates use measurements, never elapsed phase labels alone."""

from pathlib import Path

import numpy as np
import yaml

from wheel_loader.control.spatial_scoop import SpatialScoop


def setup(phase="penetrate"):
    cfg = yaml.safe_load(
        (
            Path(__file__).resolve().parents[2]
            / "configs/experiments/spatial_scoop.yaml"
        ).read_text()
    )
    c = SpatialScoop(cfg)
    c.phase = phase
    o = {
        "clearance": 0.1,
        "clearance_rate": 0.0,
        "lip_z": cfg["attack_height_m"],
        "pitch": cfg["attack_pitch_rad"],
        "penetration": 0.2,
        "active_material_contact": False,
        "distance_to_entry": 0.1,
        "mass": 0.0,
        "pile_top": 0.6,
        "rear_x": 0.0,
        "reaction_force_N": 0.0,
        "forward_speed_m_s": 0.12,
    }
    return c, o


def test_curl_requires_active_contact_and_measured_penetration():
    c, o = setup()
    c.goal(1, o)
    assert c.phase == "penetrate"
    o["active_material_contact"] = True
    o["penetration"] = 0.01
    c.goal(2, o)
    assert c.phase == "penetrate"
    o["penetration"] = c.cfg["curl_entry_depth_m"]
    c.goal(3, o)
    assert c.phase == "curl"


def test_lift_requires_measured_curl_and_payload():
    c, o = setup("curl")
    o["pitch"] = c.cfg["curl_pitch_rad"]
    c.goal(1, o)
    assert c.phase == "curl"
    o["mass"] = 17.0
    c.goal(2, o)
    assert c.phase == "lift"


def test_ground_prediction_stops_drive_before_floor_crossing():
    c, o = setup()
    assert c.goal(1, o)[2] > 0
    o["clearance_rate"] = -1.0
    assert c.goal(2, o)[2] == 0
    o["clearance"] = -0.002
    c.goal(3, o)
    assert c.failed and c.reason == "bucket_ground_clearance_violation"


def test_contact_force_limit_brakes_without_inventing_load():
    c, o = setup()
    o["reaction_force_N"] = c.cfg["breakout_threshold_N"]
    goal = c.goal(1, o)
    assert goal[2] == 0 and o["mass"] == 0 and c.phase == "penetrate"
    assert np.isfinite(goal).all()


def test_stalled_curl_still_requires_contact_and_minimum_entry():
    c, o = setup()
    o["forward_speed_m_s"] = 0.0
    o["penetration"] = c.cfg["minimum_stall_entry_m"]
    c.goal(1, o)
    c.goal(3, o)
    assert c.phase == "penetrate"
    o["active_material_contact"] = True
    c.goal(4, o)
    assert c.phase == "penetrate"
    c.goal(5.1, o)
    assert c.phase == "curl"


def test_empty_bucket_cannot_pass_retention():
    c, o = setup("retain")
    c.goal(3, o)
    assert c.failed and not c.done and c.reason == "no_retained_payload"
