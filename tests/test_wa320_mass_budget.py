"""Independent mass-allocation, compound inertia and exported-body checks."""

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from wheel_loader.vehicle.asset import build_asset
from wheel_loader.vehicle.mass_properties import body_properties, compound_properties
from wheel_loader.vehicle.parameters import load_parameters

ROOT = Path(__file__).resolve().parents[1]


def test_every_component_is_counted_once_and_export_matches(tmp_path):
    p = load_parameters(ROOT / "configs/vehicle/wa320.yaml")
    props = body_properties(p)
    assert sum(v["mass_kg"] for v in props.values()) == pytest.approx(15175, abs=1e-9)
    tree = ET.parse(build_asset(p, tmp_path))
    for name, value in props.items():
        node = tree.find(f"link[@name='{name}']/inertial")
        assert float(node.find("mass").attrib["value"]) == pytest.approx(
            value["mass_kg"]
        )
        np.testing.assert_allclose(
            np.fromstring(node.find("origin").attrib["xyz"], sep=" "),
            value["com_m"],
            atol=1e-10,
        )
        tensor = np.array(value["inertia_kg_m2"])
        assert np.linalg.eigvalsh(tensor).min() > 0
    joints = {j.attrib["name"]: j for j in tree.findall("joint")}
    assert joints["articulation_joint"].attrib["type"] == "revolute"
    assert (
        joints["engine_powertrain_fixed"].find("parent").attrib["link"] == "rear_frame"
    )
    assert joints["cab_fixed"].find("parent").attrib["link"] == "rear_frame"
    bad = copy.deepcopy(p)
    bad["bodies"]["cab"]["mass_fraction"] = 2
    with pytest.raises(ValueError, match="double counted"):
        body_properties(bad)


def test_parallel_axis_and_rotation_have_independent_analytic_values():
    parts = [
        {
            "shape": "box",
            "fraction": 0.5,
            "center": [-1, 0, 0],
            "size": [2, 4, 6],
            "rpy": [0, 0, 0],
        },
        {
            "shape": "box",
            "fraction": 0.5,
            "center": [1, 0, 0],
            "size": [2, 4, 6],
            "rpy": [0, 0, np.pi / 2],
        },
    ]
    value = compound_properties(12, parts)
    np.testing.assert_allclose(value["com_m"], [0, 0, 0])
    np.testing.assert_allclose(
        value["inertia_kg_m2"], np.diag([46, 58, 32]), atol=1e-10
    )
