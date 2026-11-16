"""Static Komodo model inspection. No ROS, Gazebo, or hardware integration."""
from pathlib import Path
import hashlib
import math
import struct
import xml.etree.ElementTree as ET
import numpy as np

UPSTREAM_REVISION = 'd53759100ced7439dd501620f955f347087e4f63'
MOVING_JOINTS = ('rear_left_wheel_joint', 'front_left_wheel_joint', 'rear_right_wheel_joint',
                 'front_right_wheel_joint', 'arm_joint', 'bucket_joint')


def expanded_model(source):
    """Expand only the description file using modern xacro; no package lookup needed."""
    import xacro
    return ET.fromstring(xacro.process_file(str(Path(source) / 'komodo2_description/urdf/komodo2.xacro')).toxml())


def vector(element, attribute, default='0 0 0'):
    return [float(value) for value in (element.get(attribute, default) if element is not None else default).split()]


def inertial_record(link):
    inertial = link.find('inertial')
    if inertial is None:
        return None
    item = inertial.find('inertia')
    values = {key: float(item.get(key, '0')) for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')}
    matrix = np.array([[values['ixx'], values['ixy'], values['ixz']],
                       [values['ixy'], values['iyy'], values['iyz']],
                       [values['ixz'], values['iyz'], values['izz']]])
    eigenvalues = np.linalg.eigvalsh(matrix)
    return {'mass_kg': float(inertial.find('mass').get('value')), 'com_m': vector(inertial.find('origin'), 'xyz'),
            'rpy_rad': vector(inertial.find('origin'), 'rpy'), 'tensor_kg_m2': values,
            'principal_moments_kg_m2': eigenvalues.tolist(),
            'positive_definite': bool(np.all(eigenvalues > 0)),
            'physical_triangle_inequality': bool(eigenvalues[2] <= eigenvalues[:2].sum() + 1e-9)}


def resolve_mesh(source, uri):
    prefix = 'package://'
    if not uri.startswith(prefix):
        raise ValueError('Expected an upstream package:// mesh URI')
    return Path(source) / uri[len(prefix):]


def inspect_model(source):
    root = expanded_model(source)
    links, joints, meshes = [], [], {}
    for link in root.findall('link'):
        geometry = {}
        for role in ('visual', 'collision'):
            geometry[role] = []
            for item in link.findall(role):
                shape = item.find('geometry')[0]
                geometry[role].append({'kind': shape.tag, 'parameters': dict(shape.attrib),
                                       'origin_xyz_m': vector(item.find('origin'), 'xyz'),
                                       'origin_rpy_rad': vector(item.find('origin'), 'rpy')})
                if shape.tag == 'mesh':
                    uri = shape.get('filename')
                    path = resolve_mesh(source, uri)
                    record = {'available': path.is_file(), 'source_package': uri.split('/')[2]}
                    if path.is_file():
                        data = path.read_bytes()
                        record.update(sha256=hashlib.sha256(data).hexdigest(), bytes=len(data))
                        count = struct.unpack_from('<I', data, 80)[0]
                        if len(data) == 84 + count * 50:
                            dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
                            triangles = np.frombuffer(data, offset=84, dtype=dtype)['vertices']
                            record.update(triangles=count, bounds_min_m=triangles.min(axis=(0, 1)).tolist(),
                                          bounds_max_m=triangles.max(axis=(0, 1)).tolist(),
                                          embedded_provenance=data[:80].decode('ascii', errors='replace').strip())
                    meshes[uri] = record
        links.append({'name': link.get('name'), 'inertial': inertial_record(link), **geometry})
    for joint in root.findall('joint'):
        joints.append({'name': joint.get('name'), 'type': joint.get('type'),
                       'parent': joint.find('parent').get('link'), 'child': joint.find('child').get('link'),
                       'origin_xyz_m': vector(joint.find('origin'), 'xyz'),
                       'origin_rpy_rad': vector(joint.find('origin'), 'rpy'),
                       'axis': vector(joint.find('axis'), 'xyz', '1 0 0'),
                       'limits': {k: float(v) for k, v in (joint.find('limit').attrib if joint.find('limit') is not None else {}).items()},
                       'dynamics': {k: float(v) for k, v in (joint.find('dynamics').attrib if joint.find('dynamics') is not None else {}).items()}})
    missing = [uri for uri, record in meshes.items() if not record['available']]
    bad = [link['name'] for link in links if link['inertial'] and not link['inertial']['physical_triangle_inequality']]
    return {'upstream': 'https://github.com/osheraz/komodo', 'revision': UPSTREAM_REVISION,
            'links': links, 'joints': joints, 'meshes': meshes, 'missing_meshes': missing,
            'invalid_inertia_links': bad, 'faithful_conversion_ready': not missing and not bad,
            'steering': {'mechanism': 'skid_steer', 'steering_joint': None, 'chassis_articulation_joint': None},
            'license': {'repository': 'MIT', 'description_package': 'TODO',
                        'mesh_origin': 'not independently established; STL header says VCG',
                        'redistribution': 'do not publish converted mesh assets pending provenance review'}}


def rpy_matrix(rpy):
    x, y, z = rpy
    cx, cy, cz, sx, sy, sz = math.cos(x), math.cos(y), math.cos(z), math.sin(x), math.sin(y), math.sin(z)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx], [-sy, cy*sx, cy*cx]])


def forward_kinematics(joints, positions=None):
    """URDF column-vector transforms: T_parent_origin * R_rpy * R_axis(q)."""
    positions = positions or {}
    frames = {'base_footprint': np.eye(4)}
    remaining = list(joints)
    while remaining:
        progressed = False
        for joint in remaining[:]:
            if joint['parent'] not in frames:
                continue
            local = np.eye(4)
            local[:3, :3] = rpy_matrix(joint['origin_rpy_rad'])
            local[:3, 3] = joint['origin_xyz_m']
            if joint['type'] != 'fixed':
                axis = np.asarray(joint['axis'], dtype=float)
                axis /= np.linalg.norm(axis)
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                angle = positions.get(joint['name'], 0.0)
                rotation = np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)
                local[:3, :3] = local[:3, :3] @ rotation
            frames[joint['child']] = frames[joint['parent']] @ local
            remaining.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError('Disconnected or cyclic joint graph')
    return frames
