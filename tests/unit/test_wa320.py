"""Meaningful checks of linkage closure, wheel kinematics and archive isolation."""

import ast
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from wheel_loader.control.commands import LoaderCommand, wheel_rates
from wheel_loader.evaluation.foundation import check_geometry
from wheel_loader.terrain.rigid_pile import select_terrain
from wheel_loader.vehicle.asset import build_asset
from wheel_loader.vehicle.kinematics import (
    circle_intersection,
    passive_joint_positions,
    rotate,
    solve_linkage,
)
from wheel_loader.vehicle.parameters import load_parameters
from wheel_loader.vehicle.registry import select_vehicle

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def p():
    return load_parameters(ROOT / "configs/vehicle/wa320.yaml")


def test_closed_linkage_through_entire_configured_envelope(p):
    result = check_geometry(p)
    assert result["samples"] == 693
    assert result["max_loop_residual_m"] < 1e-12
    assert not result["physics_validated"]
    for tilt in np.linspace(-0.8, 0.8, 25):
        pose = solve_linkage(p, 0.4, float(tilt))
        # Independently reconstruct the attachment using the passive bucket angle.
        expected = pose.bucket_pin + rotate(
            pose.boom_elevation + pose.bucket_angle, p["linkage"]["bucket_anchor"]
        )
        assert expected == pytest.approx(pose.bucket_anchor, abs=1e-12)
        assert np.linalg.norm(pose.upper_pin - pose.bellcrank_pin) == pytest.approx(
            p["linkage"]["bellcrank_upper_length"]
        )
        assert np.linalg.norm(pose.lower_pin - pose.bellcrank_pin) == pytest.approx(
            p["linkage"]["bellcrank_lower_length"]
        )


def test_bucket_is_passive_and_coupled_to_tilt_not_an_independent_action(p):
    neutral = passive_joint_positions(p)
    dump = passive_joint_positions(p, tilt_angle=0.7)
    assert neutral["bucket_joint"] == pytest.approx(0)
    assert dump["bucket_joint"] < -0.4
    assert dump["rod_joint"] != pytest.approx(0)
    # Lift at constant relative bellcrank preserves the four-bar assembly in boom frame.
    assert solve_linkage(p, 0.8).bucket_angle == pytest.approx(
        solve_linkage(p, 0).bucket_angle
    )


@pytest.mark.parametrize("args", [(0, 1.01), (1.26, 0), (float("nan"), 0)])
def test_linkage_rejects_invalid_commands(p, args):
    with pytest.raises(ValueError):
        solve_linkage(p, *args)


def test_circle_closure_rejects_unreachable_and_toggle():
    for b in ([4, 0], [2, 0], [0, 0]):
        with pytest.raises(ValueError):
            circle_intersection([0, 0], 1, b, 1)


def test_wheel_rates_match_no_slip_constraints_and_reverse(p):
    for angle in (-0.6, 0, 0.6):
        for adot in (-0.1, 0, 0.1):
            values = wheel_rates(p, 0.6, angle, adot)
            r = p["vehicle"]["wheel_radius"]
            w = p["vehicle"]["track_width"]
            l = p["vehicle"]["wheelbase"] / 2
            vr = (
                (values["rear_left_wheel_joint"] + values["rear_right_wheel_joint"])
                * r
                / 2
            )
            vf = (
                (values["front_left_wheel_joint"] + values["front_right_wheel_joint"])
                * r
                / 2
            )
            wr = (
                (values["rear_right_wheel_joint"] - values["rear_left_wheel_joint"])
                * r
                / w
            )
            wf = (
                (values["front_right_wheel_joint"] - values["front_left_wheel_joint"])
                * r
                / w
            )
            assert vr == pytest.approx(0.6)
            assert wf - wr == pytest.approx(adot)
            # Lateral velocity of the common hinge agrees in front and rear frames.
            assert -vr * np.sin(angle) + l * wr * np.cos(angle) == pytest.approx(
                -l * wf
            )
            assert vf == pytest.approx(vr * np.cos(angle) + l * wr * np.sin(angle))
        reverse = wheel_rates(p, -0.6, angle)
        forward = wheel_rates(p, 0.6, angle)
        assert list(reverse.values()) == pytest.approx([-v for v in forward.values()])


def test_generated_asset_is_repeatable_and_includes_passive_closure(p, tmp_path):
    asset = build_asset(p, tmp_path)
    assert (
        asset.read_bytes()
        == (ROOT / "assets/wheel_loader/wa320/wa320.urdf").read_bytes()
    )
    tree = ET.parse(asset)
    joints = {j.attrib["name"]: j for j in tree.findall("joint")}
    assert len(joints) == 9
    assert joints["articulation_joint"].find("axis").attrib["xyz"] == "0 0 1"
    assert joints["boom_joint"].find("axis").attrib["xyz"] == "0 -1 0"
    assert joints["bucket_joint"].find("limit").attrib["effort"] == "0"
    closure = json.loads((tmp_path / "closure.json").read_text())
    assert closure["body0"] == "rod" and closure["body1"] == "bucket"
    assert closure["exclude_from_articulation"]
    total = sum(
        float(link.find("inertial/mass").attrib["value"])
        for link in tree.findall("link")
    )
    assert total == pytest.approx(p["vehicle"]["mass"])
    for link in tree.findall("link"):
        inertia = link.find("inertial/inertia").attrib
        values = [float(inertia[k]) for k in ("ixx", "iyy", "izz")]
        assert min(values) > 0 and 2 * max(values) <= sum(values) + 1e-6


def test_configuration_rejects_mass_mismatch(p, tmp_path):
    import yaml

    p["masses"]["wheel"] = -1
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(p))
    with pytest.raises(ValueError):
        load_parameters(path)


def test_commands_and_terrain_fail_explicitly(p):
    LoaderCommand().validate(p)
    with pytest.raises(ValueError):
        LoaderCommand(articulation_angle=1).validate(p)
    assert select_terrain().vertices().shape == (8, 3)
    with pytest.raises(NotImplementedError):
        select_terrain("granular_dem")
    with pytest.raises(ValueError):
        select_terrain(height=-1)


def test_registry_references_require_opt_in_and_do_not_import_archive():
    with pytest.raises(ValueError):
        select_vehicle("komodo")
    assert not select_vehicle("komodo", allow_reference=True).active
    assert select_vehicle(root=ROOT).parameters["vehicle"]["wheelbase"] == 3.03
    assert not any(
        name.startswith(("komodo_reference", "wheel_loader_rl", "isaaclab", "isaacsim"))
        for name in sys.modules
    )
    for path in (ROOT / "src/wheel_loader").rglob("*.py"):
        tree = ast.parse(path.read_text())
        imports = [
            n.module or "" for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)
        ]
        imports += [
            alias.name
            for n in ast.walk(tree)
            if isinstance(n, ast.Import)
            for alias in n.names
        ]
        assert not any(
            n.startswith(("komodo_reference", "wheel_loader_rl", "extras"))
            for n in imports
        )


def test_training_entry_point_runs_foundation_and_refuses_ppo():
    result = subprocess.run(
        [sys.executable, "train.py", "--vehicle", "wa320"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["samples"] == 693
    blocked = subprocess.run(
        [sys.executable, "train.py", "--mode", "ppo"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert blocked.returncode != 0 and "scripted scooping" in blocked.stderr


def test_terrain_configuration_controls_geometry(tmp_path):
    from wheel_loader.terrain.rigid_pile import load_terrain

    terrain = load_terrain(ROOT / "configs/terrain/rigid_pile.yaml")
    path = tmp_path / "terrain.yaml"
    path.write_text(
        "backend: rigid_pile\nlength: 10.0\nwidth: 8.0\nheight: 4.0\ncrest_length: 3.0\n"
    )
    larger = load_terrain(path)
    assert larger.vertices()[:, 2].max() == 4.0
    assert np.ptp(larger.vertices()[:, 0]) > np.ptp(terrain.vertices()[:, 0])
    path.write_text("backend: multiscale_dem\n")
    with pytest.raises(NotImplementedError):
        load_terrain(path)
