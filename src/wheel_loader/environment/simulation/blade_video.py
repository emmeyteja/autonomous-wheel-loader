"""Replay recorded blade/material states with world-frame force telemetry."""

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np


def render_blade(folder, output):
    import mujoco
    from PIL import Image, ImageDraw, ImageFont

    folder, output = Path(folder), Path(output)
    result = json.loads((folder / "validation.json").read_text())
    if not all(result["checks"].values()):
        raise ValueError("Resolve failed numerical screens before publishing replay")
    with np.load(folder / "motion.npz") as saved:
        recording = {k: saved[k] for k in saved.files}
    times = recording["time_s"]
    intervals = np.diff(times)
    if (
        not len(intervals)
        or intervals[0] <= 0
        or not np.allclose(intervals, intervals[0], rtol=1e-6, atol=1e-8)
    ):
        raise ValueError("Need uniform replay times")
    model = mujoco.MjModel.from_xml_path(str(folder / "effective_model.xml"))
    model.vis.global_.offwidth, model.vis.global_.offheight = 840, 720
    model.vis.headlight.ambient[:] = [0.6, 0.6, 0.6]
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.15, 0, 0.12]
    camera.distance, camera.azimuth, camera.elevation = 0.95, 125, -27
    renderer = mujoco.Renderer(model, height=720, width=840)
    try:
        font = ImageFont.truetype("DejaVuSansMono.ttf", 17)
    except OSError:
        font = ImageFont.load_default()
    encoder_path = shutil.which("ffmpeg")
    if encoder_path is None:
        renderer.close()
        raise RuntimeError("Install ffmpeg")
    output.parent.mkdir(parents=True, exist_ok=True)
    phase_names = [
        "SETTLING",
        "INSERTION",
        "HOLD / SETTLE",
        "FORWARD CUT",
        "RELAXATION",
    ]
    raw_times = recording["force_time_s"]
    draft = -recording["force_world_N"][:, 0]
    ylo, yhi = min(0, float(draft.min())), max(1, float(draft.max()))
    try:
        with subprocess.Popen(
            [
                encoder_path,
                "-y",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                "1280x720",
                "-r",
                str(1 / intervals[0]),
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-crf",
                "20",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(output),
            ],
            stdin=subprocess.PIPE,
        ) as encoder:
            try:
                for i, t in enumerate(times):
                    q = data.qpos.reshape(-1, 7)
                    q[0, :3] = recording["blade_xyz"][i]
                    q[0, 3:] = recording["blade_quat_wxyz"][i]
                    q[1:, :3] = recording["xyz"][i]
                    q[1:, 3:] = [1, 0, 0, 0]
                    data.mocap_pos[0] = q[0, :3]
                    data.mocap_quat[0] = q[0, 3:]
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, camera)
                    frame = Image.new("RGB", (1280, 720))
                    frame.paste(Image.fromarray(renderer.render()), (0, 0))
                    draw = ImageDraw.Draw(frame)
                    draw.rectangle((10, 10, 1268, 86), fill=(15, 20, 28))
                    phase = phase_names[int(recording["phase"][i])]
                    draw.text(
                        (24, 22),
                        f"BLADE FORCE DIAGNOSTIC | {phase} | t={t:.2f}s | 1x recorded replay",
                        font=font,
                        fill="white",
                    )
                    draw.text(
                        (24, 53),
                        f"Uncalibrated coarse {result['material'].replace('_', ' ')} | Not a matched paper test or loader/bucket validation",
                        font=font,
                        fill=(255, 195, 80),
                    )
                    draw.rectangle((845, 105, 1268, 700), fill=(15, 20, 28))
                    j = min(int(np.searchsorted(raw_times, t)), len(raw_times) - 1)
                    force = recording["force_world_N"][j]
                    pos = recording["blade_position_m"][j]
                    vel = recording["blade_velocity_m_s"][j]
                    values = [
                        "WORLD FRAME: X travel, Z up",
                        f"Width/depth: {result['width_m'] * 1000:.0f}/{result['depth_m'] * 1000:.0f} mm",
                        f"Grains: {result['element_count']} | mass {result['represented_mass_kg']:.2f} kg",
                        "Grain rotations remain free",
                        "Blade XYZ [m]:",
                        f" {pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}",
                        "Vel XYZ [m/s]:",
                        f" {vel[0]:+.3f} {vel[1]:+.3f} {vel[2]:+.3f}",
                        "Material force on blade [N]:",
                        f" {force[0]:+.2f} {force[1]:+.2f} {force[2]:+.2f}",
                        f"Draft H=-Fx: {-force[0]:.2f} N",
                        "",
                        "FULL-RUN NUMERICAL SCREEN: PASS",
                        "Physical validation: NOT VALIDATED",
                        "FINAL CUT FIT (diagnostic window)",
                        f"H0: {result['force_fit']['H0_N']:.2f} N",
                        f"Fit RMS: {result['force_fit']['residual_rms_N']:.2f} N",
                        f"Cut work: {result['cut_work_J']:.3f} J",
                    ]
                    for row, line in enumerate(values):
                        draw.text(
                            (860, 119 + row * 23), line, font=font, fill=(225, 230, 240)
                        )
                    draw.text(
                        (860, 548),
                        "RECORDED DRAFT H [N] vs time [s]",
                        font=font,
                        fill="white",
                    )
                    draw.rectangle((865, 590, 1245, 668), outline=(110, 120, 135))
                    draw.text(
                        (868, 565), f"{yhi:.1f} N", font=font, fill=(180, 200, 215)
                    )
                    indices = np.arange(0, j + 1, max(1, (j + 1) // 750))
                    curve = [
                        (
                            865 + 380 * raw_times[k] / times[-1],
                            668 - 78 * (draft[k] - ylo) / (yhi - ylo),
                        )
                        for k in indices
                    ]
                    if len(curve) > 1:
                        draw.line(curve, fill=(255, 195, 80), width=2)
                    for fraction in (0, 0.5, 1):
                        draw.text(
                            (865 + 350 * fraction, 678),
                            f"{times[-1] * fraction:.0f}s",
                            font=font,
                            fill=(180, 200, 215),
                        )
                    encoder.stdin.write(np.asarray(frame).tobytes())
            finally:
                encoder.stdin.close()
            if encoder.wait():
                raise RuntimeError("Video encoding failed")
    finally:
        renderer.close()
    return output.resolve()
