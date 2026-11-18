"""Original procedural tire appearance, in metres with axle along local Y.

Visual detail only: dimensions are stylized, not a manufacturer tread design.
The outer envelope matches the existing rigid tire radius and width.
"""

import math
from pathlib import Path


def tire_meshes(radius, width):
    """Return named (vertices, triangles, RGB) meshes; no collision geometry."""
    meshes = {}

    def revolved(profile, segments=96):
        vertices = [
            (r * math.cos(a), y, r * math.sin(a))
            for r, y in profile
            for a in (2 * math.pi * i / segments for i in range(segments))
        ]
        faces = []
        for j in range(len(profile)):
            for i in range(segments):
                a = j * segments + i
                b = j * segments + (i + 1) % segments
                c = ((j + 1) % len(profile)) * segments + (i + 1) % segments
                d = ((j + 1) % len(profile)) * segments + i
                faces.extend([(a, c, b), (a, d, c)])
        return vertices, faces

    profile = [
        (0.46, -0.40),
        (0.68, -0.50),
        (0.88, -0.48),
        (0.95, -0.35),
        (0.96, -0.18),
        (0.965, 0),
        (0.96, 0.18),
        (0.95, 0.35),
        (0.88, 0.48),
        (0.68, 0.50),
        (0.46, 0.40),
    ]
    meshes["rubber"] = (
        *revolved([(r * radius, y * width) for r, y in profile]),
        (0.065, 0.072, 0.08),
    )
    vertices, faces = [], []
    for row in range(28):
        for side in (-1, 1):
            start = len(vertices)
            # Two swept lugs form each chevron; shallow gaps remain at the centre.
            for r in (0.945 * radius, radius):
                for y, sweep in ((0.025 * width, 0), (0.40 * width, 0.105)):
                    for offset in (-0.036, 0.036):
                        a = row * 2 * math.pi / 28 + sweep + offset
                        vertices.append((r * math.cos(a), side * y, r * math.sin(a)))
            for a, b, c, d in (
                (0, 1, 3, 2),
                (4, 6, 7, 5),
                (0, 4, 5, 1),
                (2, 3, 7, 6),
                (0, 2, 6, 4),
                (1, 5, 7, 3),
            ):
                triangle = [
                    (start + a, start + b, start + c),
                    (start + a, start + c, start + d),
                ]
                faces.extend(
                    triangle if side > 0 else [tuple(reversed(f)) for f in triangle]
                )
    meshes["tread"] = (vertices, faces, (0.095, 0.103, 0.11))
    rim = [
        (0.17, -0.405),
        (0.40, -0.405),
        (0.465, -0.445),
        (0.48, -0.42),
        (0.44, -0.35),
        (0.44, 0.35),
        (0.48, 0.42),
        (0.465, 0.445),
        (0.40, 0.405),
        (0.17, 0.405),
    ]
    meshes["rim"] = (
        *revolved([(r * radius, y * width) for r, y in rim]),
        (0.95, 0.58, 0.08),
    )
    meshes["hub"] = (
        *revolved(
            [
                (0, -0.46 * width),
                (0.17 * radius, -0.46 * width),
                (0.17 * radius, 0.46 * width),
                (0, 0.46 * width),
            ],
            48,
        ),
        (0.27, 0.29, 0.31),
    )
    vertices, faces = [], []
    for side in (-1, 1):
        for i in range(10):
            a = 2 * math.pi * i / 10
            points, triangles = revolved(
                [
                    (0, -0.012 * width),
                    (0.026 * radius, -0.012 * width),
                    (0.026 * radius, 0.012 * width),
                    (0, 0.012 * width),
                ],
                6,
            )
            offset = len(vertices)
            vertices.extend(
                (
                    x + 0.285 * radius * math.cos(a),
                    y + side * 0.42 * width,
                    z + 0.285 * radius * math.sin(a),
                )
                for x, y, z in points
            )
            faces.extend(tuple(v + offset for v in f) for f in triangles)
    meshes["bolts"] = (vertices, faces, (0.55, 0.57, 0.59))
    return meshes


def write_tire_meshes(radius, width, folder):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    for name, (vertices, faces, _) in tire_meshes(radius, width).items():
        lines = ["# Original procedural visual mesh; axle Y; units metres"]
        lines.extend("v " + " ".join(f"{v:.8g}" for v in point) for point in vertices)
        lines.extend("f " + " ".join(str(i + 1) for i in face) for face in faces)
        (folder / f"tire_{name}.obj").write_text("\n".join(lines) + "\n")
