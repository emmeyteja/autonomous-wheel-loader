"""Display the authored approximation using recorded rigid-body poses."""
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from wheel_loader_rl.material_accounting import bucket_local_positions


def loader_faces(poses, names):
    """Return coarse visual faces, transformed by physics-recorded body poses."""
    source = ET.parse(Path(__file__).parents[1] / 'assets/loader_approximation.xml').getroot()
    faces, colors = [], []
    indices = ((0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5))
    for body in source.iter('body'):
        pose = poses[list(names).index(body.get('name'))]
        # Transform local vertices via the inverse of the world->local function.
        rotation = bucket_local_positions(np.eye(3) + pose[:3], pose)
        for geom in body.findall('geom'):
            kind = geom.get('type')
            size = np.fromstring(geom.get('size'), sep=' ')
            center = np.fromstring(geom.get('pos', '0 0 0'), sep=' ')
            color = tuple(np.fromstring(geom.get('rgba', '0.9 0.6 0.08 1'), sep=' '))
            if kind == 'cylinder':
                angles = np.arange(12) * 2 * np.pi / 12
                rings = [np.column_stack((size[0] * np.cos(angles), np.full(12, sign * size[1]), size[0] * np.sin(angles))) + center for sign in (-1, 1)]
                local_faces = [rings[0], rings[1]] + [np.array([rings[0][i], rings[0][(i+1)%12], rings[1][(i+1)%12], rings[1][i]]) for i in range(12)]
            else:
                if kind == 'capsule':
                    ends = np.fromstring(geom.get('fromto'), sep=' ').reshape(2, 3)
                    center = ends.mean(axis=0)
                    size = abs(ends[1] - ends[0]) / 2 + size[0]
                vertices = np.array(list(product((-1, 1), repeat=3))) * size + center
                local_faces = [vertices[list(ids)] for ids in indices]
            for face in local_faces:
                faces.append(face @ rotation.T + pose[:3])
                colors.append(color)
    return faces, colors
