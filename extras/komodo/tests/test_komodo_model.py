"""Regression checks for the audited source facts and conversion refusal rules."""
import json
from pathlib import Path
import numpy as np
import pytest
from komodo_reference.inspection import forward_kinematics, MOVING_JOINTS, inertial_record
import xml.etree.ElementTree as ET

MAPPING = json.loads(Path('extras/komodo/docs/komodo_mapping.json').read_text())


def test_source_topology_and_limit_contracts():
    joints = {joint['name']: joint for joint in MAPPING['joints']}
    assert len(MAPPING['links']) == 19 and len(joints) == 18
    assert {name for name, joint in joints.items() if joint['type'] != 'fixed'} == set(MOVING_JOINTS)
    assert joints['arm_joint']['limits'] == {'effort': 100., 'lower': -.1, 'upper': .32, 'velocity': .2}
    assert joints['bucket_joint']['limits']['velocity'] == .01
    assert MAPPING['steering']['chassis_articulation_joint'] is None
    assert len(MAPPING['missing_meshes']) == 4
    assert len(MAPPING['invalid_inertia_links']) == 4
    assert not MAPPING['faithful_conversion_ready']


def test_bucket_hinge_depends_on_arm_not_curl():
    zero = forward_kinematics(MAPPING['joints'])
    curl = forward_kinematics(MAPPING['joints'], {'bucket_joint': .5})
    lift = forward_kinematics(MAPPING['joints'], {'arm_joint': .2})
    assert zero['bucket'][:3, 3] == pytest.approx(curl['bucket'][:3, 3])
    assert not np.allclose(zero['bucket'][:3, :3], curl['bucket'][:3, :3])
    assert np.linalg.norm(lift['bucket'][:3, 3]-zero['bucket'][:3, 3]) > .1
    assert lift['bucket'][2, 3] < zero['bucket'][2, 3]
    # The arm extends toward base_link +X, which is footprint -X after pi yaw.
    assert zero['bucket'][0, 3] < zero['base_link'][0, 3]
    assert zero['front_left_wheel_link'][1, 3] == pytest.approx(-.18955)


def test_invalid_wheel_tensor_is_positive_but_not_physically_possible():
    link = ET.fromstring('<link><inertial><mass value="1.8524"/><inertia ixx=".0069046" iyy=".17" izz=".0069054"/></inertial></link>')
    result = inertial_record(link)
    assert result['positive_definite']
    assert not result['physical_triangle_inequality']


def test_disconnected_joint_graph_is_rejected():
    joints = [{'parent': 'missing', 'child': 'child', 'name': 'bad', 'type': 'fixed',
               'origin_xyz_m': [0,0,0], 'origin_rpy_rad': [0,0,0]}]
    with pytest.raises(ValueError, match='Disconnected'):
        forward_kinematics(joints)


def test_strict_preparation_refuses_incomplete_source_before_writing(monkeypatch, tmp_path):
    from komodo_reference import prepare
    monkeypatch.setattr(prepare, 'inspect_model', lambda source: {'faithful_conversion_ready': False})
    destination = tmp_path/'refused'
    with pytest.raises(ValueError, match='Faithful Komodo conversion blocked'):
        prepare.prepare_model(tmp_path, destination)
    assert not destination.exists()


def test_uniform_cylinder_repair_is_physically_consistent():
    m, r, length = 1.8524, .127, .05
    cross = m*(3*r*r+length*length)/12
    axle = m*r*r/2
    moments = sorted([cross, axle, cross])
    assert moments[0] > 0
    assert moments[-1] <= sum(moments[:2])
