"""Replay recorded CG/Newton diagnostic qpos; no simulation steps or pose correction."""

import argparse
import os
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from wheel_loader.environment.simulation.surface_visual import (
    apply_surface_texture,
    surface_xml,
)
from wheel_loader.environment.simulation.tire_visual import replay_xml_with_tires
from wheel_loader.environment.simulation.vehicle_visual import apply_loader_palette


def render(root):
    output = root / "unloaded_solver_comparison.mp4"
    font = ImageFont.truetype("DejaVuSans.ttf", 21)
    proc = subprocess.Popen(
        [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            "1600x1080",
            "-framerate",
            "20",
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            str(output),
        ],
        stdin=subprocess.PIPE,
    )
    try:
        for fixture, title in [
            ("fixed", "Rear chassis fixed; all contacts disabled"),
            ("ground", "Floating loader on ground; no pile"),
        ]:
            panels = []
            for variant, solver in [(fixture, "CG"), (fixture + "_newton", "Newton")]:
                folder = root / variant
                model = mujoco.MjModel.from_xml_string(
                    surface_xml(
                        replay_xml_with_tires(folder / "bundle/model.xml"), "asphalt"
                    )
                )
                apply_surface_texture(model, "asphalt")
                model.vis.global_.offwidth, model.vis.global_.offheight = 800, 450
                model.vis.headlight.ambient[:] = [0.65, 0.65, 0.65]
                apply_loader_palette(model)
                renderer = mujoco.Renderer(model, height=450, width=800)
                for engine in ["native", "mjwarp"]:
                    states = dict(np.load(folder / engine / "states.npz"))
                    panels.append(
                        (model, renderer, mujoco.MjData(model), states, solver, engine)
                    )
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [0.9, 0, 1.2]
            camera.distance, camera.azimuth, camera.elevation = 9.0, 90.0, -13.0
            try:
                for t in np.arange(
                    0, 3, 0.025
                ):  # Half-speed playback of the actual three-second traces.
                    frame = Image.new("RGB", (1600, 1080), (20, 24, 30))
                    draw = ImageDraw.Draw(frame)
                    draw.text(
                        (20, 8),
                        title + " | recorded qpos replay at 0.5x",
                        font=font,
                        fill="white",
                    )
                    draw.text(
                        (20, 38),
                        "Same physical effort tape | no material | sim-to-sim, not machine accuracy",
                        font=font,
                        fill="white",
                    )
                    for k, (model, renderer, data, n, solver, engine) in enumerate(
                        panels
                    ):
                        i = min(np.searchsorted(n["time_s"], t), len(n["time_s"]) - 1)
                        data.qpos[:] = n["qpos"][i]
                        mujoco.mj_forward(model, data)
                        renderer.update_scene(data, camera)
                        x, y = (k % 2) * 800, 90 + (k // 2) * 490
                        frame.paste(Image.fromarray(renderer.render()), (x, y))
                        draw.rectangle((x, y, x + 800, y + 36), fill=(20, 24, 30))
                        draw.text(
                            (x + 15, y + 5),
                            f"{solver} / {engine} | t={n['time_s'][i]:.2f} s | bucket Z={n['bucket_trajectory'][i, 2]:.4f} m",
                            font=font,
                            fill="white",
                        )
                    draw.text(
                        (20, 1043),
                        "Diagnostic comparison only; existing M2 baseline unchanged; physical validation remains incomplete",
                        font=font,
                        fill="white",
                    )
                    proc.stdin.write(np.asarray(frame).tobytes())
            finally:
                for renderer in {id(p[1]): p[1] for p in panels}.values():
                    renderer.close()
    finally:
        proc.stdin.close()
    if proc.wait():
        raise RuntimeError("Replay encoding failed")
    return output


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--root", type=Path, default=Path("outputs/wa320/mechanics_benchmark")
    )
    print(render(p.parse_args().root).resolve())
