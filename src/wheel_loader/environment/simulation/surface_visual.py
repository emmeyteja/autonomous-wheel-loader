"""Original procedural ground appearance; no change to rigid contact physics."""

import xml.etree.ElementTree as ET

import numpy as np

SURFACES = ("offroad", "asphalt")


def surface_texture(name, size=512):
    if name not in SURFACES:
        raise ValueError(f"Unknown surface: {name}")
    rng = np.random.default_rng(3841)
    # Multi-scale variation and aggregate flecks, generated without external images.
    if not isinstance(size, int) or size < 32:
        raise ValueError("Texture resolution must be an integer of at least 32")
    grid = rng.normal(size=(17, 17))
    grid[-1, :] = grid[0, :]
    grid[:, -1] = grid[:, 0]
    coords = np.linspace(0, 16, size, endpoint=False)
    cells = coords.astype(int)
    blend = coords - cells
    coarse = (
        (1 - blend[:, None])
        * (1 - blend[None, :])
        * grid[cells[:, None], cells[None, :]]
        + blend[:, None]
        * (1 - blend[None, :])
        * grid[cells[:, None] + 1, cells[None, :]]
        + (1 - blend[:, None])
        * blend[None, :]
        * grid[cells[:, None], cells[None, :] + 1]
        + blend[:, None] * blend[None, :] * grid[cells[:, None] + 1, cells[None, :] + 1]
    )
    fine = rng.normal(size=(size, size))
    if name == "offroad":
        base = np.array([126, 103, 73])
        value = coarse * 23 + fine * 12
        gravel = rng.random((size, size)) > 0.985
        value += gravel * 35
    else:
        base = np.array([65, 68, 71])
        value = coarse * 2 + fine * 8
        value += (rng.random((size, size)) > 0.975) * 30
    return np.clip(base + value[:, :, None], 0, 255).astype(np.uint8)


def surface_xml(xml, surface):
    if surface not in SURFACES:
        raise ValueError(f"Unknown surface: {surface}")
    root = ET.fromstring(xml)
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "texture",
        name="ground_appearance",
        type="2d",
        builtin="flat",
        width="512",
        height="512",
        rgb1=".5 .5 .5",
    )
    ET.SubElement(
        asset,
        "material",
        name="ground_appearance",
        texture="ground_appearance",
        texrepeat=".25 .25",
        texuniform="true",
        reflectance="0",
        specular=".05",
        shininess=".1",
    )
    for geom in root.findall("./worldbody/geom"):
        if geom.get("type") == "plane":
            geom.set("material", "ground_appearance")
            geom.set("rgba", "1 1 1 1")
    return ET.tostring(root, encoding="unicode")


def apply_surface_texture(model, surface):
    import mujoco

    texture = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, "ground_appearance")
    start = model.tex_adr[texture]
    pixels = surface_texture(surface)
    model.tex_data[start : start + pixels.size] = pixels.ravel()
