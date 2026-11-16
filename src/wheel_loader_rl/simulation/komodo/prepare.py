"""Prepare a strictly checked source model or an explicitly labelled diagnostic URDF."""
from copy import deepcopy
from pathlib import Path
import json
import shutil
import xml.etree.ElementTree as ET
from .inspection import expanded_model, inspect_model, resolve_mesh


def add_box(link, xyz, size, rpy='0 0 0', color='0.85 0.55 0.08 1'):
    for role in ('visual', 'collision'):
        item = ET.SubElement(link, role)
        ET.SubElement(item, 'origin', xyz=xyz, rpy=rpy)
        geometry = ET.SubElement(item, 'geometry')
        ET.SubElement(geometry, 'box', size=size)
        if role == 'visual':
            material = ET.SubElement(item, 'material', name='diagnostic_'+link.get('name'))
            ET.SubElement(material, 'color', rgba=color)


def prepare_model(source, output, diagnostic=False):
    """Default refuses missing meshes/unphysical inertias; never silently substitutes."""
    source, output = Path(source), Path(output)
    report = inspect_model(source)
    if not diagnostic and not report['faithful_conversion_ready']:
        raise ValueError('Faithful Komodo conversion blocked: missing arm/sensor meshes and invalid wheel inertias; inspect report. Diagnostic mode is a different model.')
    root = deepcopy(expanded_model(source))
    root.set('name', 'komodo_diagnostic' if diagnostic else 'komodo_source')
    changes = []
    if diagnostic:
        keep = {'base_footprint', 'base_link', 'base_arm', 'arm', 'bucket'} | {
            role+'_wheel_link' for role in ('front_left', 'front_right', 'rear_left', 'rear_right')}
        for link in list(root.findall('link')):
            if link.get('name') not in keep:
                root.remove(link)
                changes.append({'change': 'remove_sensor_frame', 'link': link.get('name')})
        for joint in list(root.findall('joint')):
            if joint.find('parent').get('link') not in keep or joint.find('child').get('link') not in keep:
                root.remove(joint)
        for link in root.findall('link'):
            name = link.get('name')
            if name in ('base_arm', 'arm', 'bucket'):
                for role in ('visual', 'collision'):
                    for item in list(link.findall(role)):
                        link.remove(item)
                changes.append({'change': 'original_primitive_proxy_for_missing_mesh', 'link': name,
                                'warning': 'not original Komodo geometry; no excavation accuracy claim'})
                if name == 'base_arm':
                    add_box(link, '0 .06 .08', '.2 .14 .2', color='.12 .15 .18 1')
                elif name == 'arm':
                    for x in (-.09, .09):
                        add_box(link, f'{x} .315615 -.11159', '.025 .66952 .035', '-.33987 0 0')
                else:
                    add_box(link, '0 .12 -.08', '.32 .24 .008')
                    add_box(link, '0 0 -.005', '.32 .008 .15')
                    for x in (-.16, .16):
                        add_box(link, f'{x} .12 -.005', '.008 .24 .15')
            if name.endswith('_wheel_link'):
                # Uniform-cylinder inertia is a diagnostic assumption, not recovered CAD data.
                mass, radius, length = 1.8524, .127, .05
                transverse = mass*(3*radius**2 + length**2)/12
                axle = mass*radius**2/2
                inertia = link.find('inertial/inertia')
                inertia.attrib.update(ixx=str(transverse), iyy=str(axle), izz=str(transverse), ixy='0', ixz='0', iyz='0')
                link.find('inertial/origin').set('xyz', '0 0 0')
                changes.append({'change': 'replace_invalid_inertia_with_uniform_cylinder', 'link': name,
                                'diagonal_kg_m2': [transverse, axle, transverse], 'com_m': [0, 0, 0]})
    output.mkdir(parents=True, exist_ok=True)
    mesh_dir = output/'meshes'
    mesh_dir.mkdir(exist_ok=True)
    for mesh in root.iter('mesh'):
        original = resolve_mesh(source, mesh.get('filename'))
        if not original.is_file():
            raise ValueError('Missing mesh; refusing incomplete conversion')
        destination = mesh_dir/original.name
        shutil.copyfile(original, destination)
        mesh.set('filename', str(destination.resolve()))
    urdf = output/(root.get('name')+'.urdf')
    ET.indent(root)
    ET.ElementTree(root).write(urdf, encoding='utf-8', xml_declaration=True)
    shutil.copyfile(source/'LICENSE', output/'UPSTREAM_LICENSE.txt')
    manifest = {'revision': report['revision'], 'variant': root.get('name'), 'original_mesh_redistribution': 'not cleared',
                'faithful_conversion': not diagnostic, 'changes': changes,
                'excluded_runtime': ['ROS', 'Gazebo', 'hardware interfaces', 'legacy controllers', 'TensorFlow']}
    (output/'preparation.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return urdf
