"""Presentation of reconstructed state and visual-only cylinder assemblies."""

import numpy as np

from .replay_state import rpy_degrees


def draw_cylinders(scene, state, initial_lengths, data, rod_body, rod_length):
    import mujoco

    def cylinder(a, b, radius, color):
        if scene.ngeom >= scene.maxgeom:
            raise RuntimeError("Replay decoration exceeds scene capacity")
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CYLINDER,
            np.zeros(3),
            np.zeros(3),
            np.eye(3).ravel(),
            np.array(color, dtype=np.float32),
        )
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CYLINDER, radius, a, b)
        geom.shininess = 0.7
        scene.ngeom += 1

    yellow, chrome, dark = (
        [0.94, 0.56, 0.07, 1],
        [0.72, 0.76, 0.8, 1],
        [0.14, 0.16, 0.18, 1],
    )
    axis = state["pin_axis"]
    for name, (base, tip) in state["pins"].items():
        direction = (tip - base) / state["lengths"][name]
        barrel_length = 0.60 * initial_lengths[name]
        radius = 0.095 if name.startswith("Lift") else 0.11
        end = base + direction * barrel_length
        cylinder(base + direction * 0.10, end, radius, yellow)
        cylinder(end - direction * 0.015, tip - direction * 0.07, radius * 0.48, chrome)
        cylinder(end - direction * 0.07, end + direction * 0.025, radius * 1.08, dark)
        cylinder(base + direction * 0.10, base + direction * 0.18, radius * 1.08, dark)
        for pin in (base, tip):
            cylinder(pin - axis * 0.065, pin + axis * 0.065, radius * 1.05, yellow)
            cylinder(pin - axis * 0.075, pin + axis * 0.075, radius * 0.45, chrome)
    # The grey member is a rigid Z-bar rod, not a telescoping hydraulic actuator.
    origin, rotation = data.xpos[rod_body], data.xmat[rod_body].reshape(3, 3)
    end = origin + rotation @ np.array([rod_length, 0, 0])
    cylinder(origin, end, 0.045, dark)
    for pin in (origin, end):
        pin_axis = rotation[:, 1]
        cylinder(pin - pin_axis * 0.065, pin + pin_axis * 0.065, 0.095, dark)
        cylinder(pin - pin_axis * 0.075, pin + pin_axis * 0.075, 0.04, chrome)


def draw_state_panel(frame, state, names, angles):
    from PIL import ImageDraw, ImageFont

    draw = ImageDraw.Draw(frame)
    try:
        font = ImageFont.truetype("DejaVuSansMono.ttf", 17)
        heading = ImageFont.truetype("DejaVuSansMono.ttf", 19)
    except OSError:
        font = ImageFont.load_default()
        heading = font
    draw.rectangle((1280, 0, 1919, 1079), fill=(15, 20, 28))
    y = 17

    def line(text, color=(215, 225, 235), title=False):
        nonlocal y
        draw.text((1298, y), text, fill=color, font=heading if title else font)
        y += 23

    def vector(value):
        return " ".join(f"{v:+8.3f}" for v in value)

    accent = (255, 193, 74)
    line("CURRENT STATE | RECORDED MOTION", accent, True)
    line("World W: X forward at start, Y left, Z up")
    line("Velocities: finite differences in simulation time")
    line("Pose axes XYZ; RPY degrees; omega rad/s")
    y += 7
    for label, (xyz, rotation) in state["frames"].items():
        line(label, accent)
        line("  XYZ [m]   " + vector(xyz))
        line("  RPY [deg] " + vector(rpy_degrees(rotation)))
        velocity, omega = state["velocities"][label]
        line("  v_W [m/s] " + vector(velocity))
        line("  omega_W   " + vector(omega))
        y += 5
    line("RELATIVE JOINTS: angle [deg] / rate [deg/s]", accent)
    for name, label in (
        ("articulation_joint", "Articulation"),
        ("boom_joint", "Boom lift"),
        ("tilt_joint", "Bellcrank"),
        ("bucket_joint", "Bucket (passive)"),
    ):
        i = names.index(name)
        line(
            f"  {label:18s} {np.degrees(angles[i]):+8.2f} / {np.degrees(state['joint_rates'][i]):+7.2f}"
        )
    y += 7
    line("CYLINDERS: ESTIMATED VISUAL GEOMETRY", accent)
    line("L = pin distance; dL = change since first sample")
    for name, length in state["lengths"].items():
        line(f"{name}: L={length:.3f}m dL={state['length_changes'][name]:+.3f}m")
        line(f"  extension rate={state['length_rates'][name]:+.3f}m/s")
        base, tip = state["pins"][name]
        line("  base_W    " + vector(base))
        line("  rod pin_W " + vector(tip))
        y += 3
    y += 7
    line("Pressure / flow / cylinder force: NOT MODELED", accent)
    line("Absolute piston stroke / limits: UNKNOWN", accent)
    line("Actuation: equivalent rotary joint drives")
    line("Rigid connecting rod is NOT a piston")
