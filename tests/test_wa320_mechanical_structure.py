"""Independent URDF forward kinematics and physical cylinder/frame structure."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from wheel_loader.evaluation.mechanical_signature import linkage_signature
from wheel_loader.vehicle.asset import build_asset
from wheel_loader.vehicle.kinematics import passive_joint_positions
from wheel_loader.vehicle.mass_properties import (
    body_transforms,
    compound_properties,
    rotation3,
)
from wheel_loader.vehicle.parameters import load_parameters

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("angle", [0, 20, -20, 38.5, -38.5])
def test_urdf_frames_cylinders_and_axles_are_physically_connected(tmp_path, angle):
    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    tree = ET.parse(build_asset(p, tmp_path))
    art = np.deg2rad(angle)
    q = passive_joint_positions(p, 0.15, -0.05, articulation=art) | {
        "articulation_joint": art
    }
    frames = {
        "rear_frame": (np.array([0, 0, p["vehicle"]["articulation_height"]]), np.eye(3))
    }
    pending = list(tree.findall("joint"))
    while pending:
        ready = [j for j in pending if j.find("parent").get("link") in frames]
        assert ready
        for j in ready:
            xyz, r = frames[j.find("parent").get("link")]
            o = j.find("origin")
            pos = np.fromstring(o.get("xyz", "0 0 0"), sep=" ")
            rot = rotation3(np.fromstring(o.get("rpy", "0 0 0"), sep=" "))
            v = q.get(j.get("name"), 0.0)
            kind = j.get("type")
            if kind != "fixed":
                axis = np.fromstring(j.find("axis").get("xyz"), sep=" ")
                if kind == "prismatic":
                    pos += rot @ axis * v
                else:
                    skew = np.array(
                        [
                            [0, -axis[2], axis[1]],
                            [axis[2], 0, -axis[0]],
                            [-axis[1], axis[0], 0],
                        ]
                    )
                    rot = rot @ (
                        np.eye(3) + np.sin(v) * skew + (1 - np.cos(v)) * (skew @ skew)
                    )
            frames[j.find("child").get("link")] = (xyz + r @ pos, r @ rot)
            pending.remove(j)
    expected = body_transforms(p, 0.15, -0.05, art)
    for name, (xyz, r) in expected.items():
        if name == "world":
            continue
        np.testing.assert_allclose(frames[name][0], xyz, atol=1e-10)
        np.testing.assert_allclose(frames[name][1], r, atol=1e-10)
    np.testing.assert_allclose(
        frames["rear_frame"][0], frames["front_frame"][0], atol=1e-12
    )
    assert tree.find("joint[@name='articulation_joint']").get("type") == "revolute"
    assert tree.find("joint[@name='articulation_joint']/axis").get("xyz") == "0 0 1"
    for name, spec in p["cylinder_bodies"].items():
        assert (
            tree.find(f"joint[@name='{name}_slider_joint']").get("type") == "prismatic"
        )
        b, r = frames[spec["tip_body"]]
        np.testing.assert_allclose(
            frames[name + "_rod"][0], b + r @ spec["tip"], atol=1e-10
        )
        assert (
            -1e-9
            <= q[name + "_slider_joint"]
            + spec["reference_length"]
            - p["cylinders"][spec["kind"]]["pin_distance_min"]
            <= p["cylinders"][spec["kind"]]["stroke"] + 1e-9
        )
    for axle, sign in [("rear", -1), ("front", 1)]:
        xyz, r = frames[axle + "_frame"]
        pos, _ = frames[axle + "_axle"]
        np.testing.assert_allclose(
            r.T @ (pos - xyz),
            [sign * 3.030 / 2, 0, 0.695 - p["vehicle"]["articulation_height"]],
            atol=1e-10,
        )
        for side, sgn in [("left", 1), ("right", -1)]:
            wp, _ = frames[f"{axle}_{side}_wheel"]
            np.testing.assert_allclose(
                r.T @ (wp - pos), [0, sgn * 2.050 / 2, 0], atol=1e-10
            )
            assert wp[2] == pytest.approx(0.695)


def test_signature_closes_over_range_and_uses_manual_cylinder_envelopes():
    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    rows = linkage_signature(p)
    assert len(rows) == 51
    assert max(r["zbar_gap_m"] for r in rows) < 1e-12
    assert max(r["maximum_cylinder_pin_gap_m"] for r in rows) < 1e-12


def test_ring_inertia_preserves_bore_and_physical_mass():
    prop = compound_properties(
        10,
        [
            {
                "shape": "annulus",
                "fraction": 1.0,
                "center": [0, 0, 0],
                "size": [0.3, 0.1, 0.5],
                "rpy": [0, 0, 0],
            }
        ],
    )
    np.testing.assert_allclose(
        prop["inertia_kg_m2"],
        np.diag([10 * (3 * 0.10 + 0.25) / 12] * 2 + [10 * 0.10 / 2]),
    )


def test_neutral_assembly_closes_before_import_and_rejects_offset(tmp_path):
    import yaml

    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    q = passive_joint_positions(p, 0, 0)
    for name in p["cylinder_bodies"]:
        assert abs(q[name + "_slider_joint"]) < 1e-12
        assert abs(q[name + "_hinge_joint"]) < 1e-12
    raw = yaml.safe_load((ROOT / "configs/vehicle/wa320.yaml").read_text())
    raw["cylinder_bodies"]["bucket"]["reference_length"]["value"] += 0.01
    file = tmp_path / "wrong-neutral.yaml"
    file.write_text(yaml.safe_dump(raw, sort_keys=False))
    with pytest.raises(ValueError, match="neutral cylinder pin length"):
        load_parameters(file)
