"""Replay actual element positions with explicit diagnostic/calibration labels."""

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np


def render_column(folder, output=None):
    import mujoco
    from PIL import Image, ImageDraw, ImageFont

    folder = Path(folder)
    output = Path(output) if output else folder / "column_replay.mp4"
    result = json.loads((folder / "validation.json").read_text())
    if not all(result["checks"].values()):
        raise ValueError("Resolve failed numerical checks before publishing a replay")
    with np.load(folder / "motion.npz") as recorded:
        times, xyz, velocity, radii, energy = [
            recorded[k]
            for k in ("time_s", "xyz", "linear_velocity", "radii", "kinetic_energy_J")
        ]
    model = mujoco.MjModel.from_xml_path(str(folder / "effective_model.xml"))
    model.vis.global_.offwidth, model.vis.global_.offheight = 840, 720
    model.vis.headlight.ambient[:] = [0.6, 0.6, 0.6]
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, 0.09]
    camera.distance, camera.azimuth, camera.elevation = 1.3, 125, -27
    renderer = mujoco.Renderer(model, height=720, width=840)
    try:
        font = ImageFont.truetype("DejaVuSansMono.ttf", 18)
    except OSError:
        font = ImageFont.load_default()
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        renderer.close()
        raise RuntimeError("Install ffmpeg")
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with subprocess.Popen(
            [
                ffmpeg,
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
                "25",
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
                    q[:, :3], q[:, 3:] = xyz[i], [1, 0, 0, 0]
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, camera)
                    frame = Image.new("RGB", (1280, 720))
                    frame.paste(Image.fromarray(renderer.render()), (0, 0))
                    draw = ImageDraw.Draw(frame)
                    draw.rectangle((10, 10, 1268, 85), fill=(15, 20, 28))
                    draw.text(
                        (24, 22),
                        f"{result['material_candidate'].upper()} CANDIDATE | COLUMN RELEASE | t={t:.2f}s | 1x recorded replay",
                        font=font,
                        fill="white",
                    )
                    draw.text(
                        (24, 53),
                        "Coarse contact-law diagnostic | NOT calibrated material | no loader/bucket",
                        font=font,
                        fill=(255, 195, 80),
                    )
                    draw.rectangle((845, 105, 1268, 665), fill=(15, 20, 28))
                    values = [
                        "MEASURED DIAGNOSTIC STATE",
                        f"Elements: {len(radii)}",
                        f"Radius: {radii.min() * 1000:.1f}-{radii.max() * 1000:.1f} mm",
                        f"Mass: {result['represented_mass_kg']:.3f} kg",
                        f"Height: {np.max(xyz[i, :, 2] + radii) * 1000:.1f} mm",
                        f"R95: {np.quantile(np.linalg.norm(xyz[i, :, :2], axis=1), 0.95) * 1000:.1f} mm",
                        f"Speed p95: {np.quantile(np.linalg.norm(velocity[i], axis=1), 0.95):.4f} m/s",
                        f"Kinetic energy: {energy[i]:.5f} J",
                        "",
                        "FULL-RUN NUMERICAL SCREEN: PASS",
                        "Physical calibration: NOT VALIDATED",
                        "Wall-free release; no locked grains",
                        "No added joint velocity damping",
                        "",
                        "FINAL SECTOR SURFACE FITS",
                    ]
                    for name, fit in result["surface_sector_fits"].items():
                        values.append(
                            f"{name}: unavailable"
                            if fit is None
                            else f"{name}: {fit['angle_deg']:.1f} deg, R2={fit['r_squared']:.2f}"
                        )
                    values += [
                        "",
                        "Sparse fits are not invented.",
                        "Different apparatus from Aoshima.",
                        "No field-test accuracy claim.",
                    ]
                    for row, text in enumerate(values):
                        draw.text(
                            (860, 120 + row * 23), text, font=font, fill=(225, 230, 240)
                        )
                    encoder.stdin.write(np.asarray(frame).tobytes())
            finally:
                encoder.stdin.close()
            if encoder.wait():
                raise RuntimeError("Video encoding failed")
    finally:
        renderer.close()
    return output.resolve()
