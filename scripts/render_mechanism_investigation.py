"""Replay full Newton-solver CPU/GPU recorded states; no physics stepping."""

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
    folder = root / "full_newton"
    model = mujoco.MjModel.from_xml_string(
        surface_xml(replay_xml_with_tires(folder / "bundle/model.xml"), "asphalt")
    )
    apply_surface_texture(model, "asphalt")
    apply_loader_palette(model)
    # Replay-only appearance: the physical plane is infinite; render a broad patch
    # so the original 5 m display extent does not make grains appear suspended.
    for g in range(model.ngeom):
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            model.geom_size[g, :2] = 100
    model.vis.global_.offwidth, model.vis.global_.offheight = 800, 500
    model.vis.headlight.ambient[:] = [0.65, 0.65, 0.65]
    renderer = mujoco.Renderer(model, height=500, width=800)
    data = mujoco.MjData(model)
    states = [dict(np.load(folder / f"{e}_1/states.npz")) for e in ["native", "mjwarp"]]
    spatial = [
        dict(np.load(root / "spatial" / e / "spatial_trace.npz"))
        for e in ["native", "mjwarp"]
    ]
    for state, trace in zip(states, spatial):
        if not np.array_equal(state["time_s"], trace["time_s"]):
            raise ValueError("Spatial audit time does not match replay")
    camera = mujoco.MjvCamera()
    camera.distance = 12
    camera.azimuth = 100
    camera.elevation = -18
    rear = next(
        i
        for i in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").endswith(
            "_rear_frame"
        )
    )
    output = root / "failed_scoop_spatial_audit_yellow.mp4"
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
            "1600x740",
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
    font = ImageFont.truetype("DejaVuSans.ttf", 21)
    try:
        for t in np.arange(0, states[0]["time_s"][-1], 0.05):
            frame = Image.new("RGB", (1600, 740), (20, 24, 30))
            draw = ImageDraw.Draw(frame)
            draw.text(
                (15, 8),
                "Recorded FAILED scoop | MuJoCo Newton CPU/GPU | original motion, 1x",
                font=font,
                fill="white",
            )
            draw.text(
                (15, 37),
                "Curl above pile; bucket intersects floor | motion controller has not been corrected",
                font=font,
                fill="white",
            )
            for k, (engine, s) in enumerate(
                zip(["Native MuJoCo", "Direct MuJoCo-Warp"], states)
            ):
                i = min(np.searchsorted(s["time_s"], t), len(s["time_s"]) - 1)
                data.qpos[:] = s["qpos"][i]
                mujoco.mj_forward(model, data)
                camera.lookat[:] = s["body_positions"][i, rear] + np.array(
                    [1.5, 0, 0.4]
                )
                renderer.update_scene(data, camera)
                frame.paste(Image.fromarray(renderer.render()), (k * 800, 100))
                draw.text(
                    (k * 800 + 15, 75),
                    f"{engine} | t={s['time_s'][i]:.2f} s | {s['phase'][i]}",
                    font=font,
                    fill="white",
                )
                xy = s["body_positions"][i, rear, :2]
                draw.text(
                    (k * 800 + 15, 675),
                    f"Rear XY=({xy[0]:.3f}, {xy[1]:.3f}) m | tracking camera",
                    font=font,
                    fill="white",
                )
                col = list(s["accounting_columns"]).index("bucket_mass_kg")
                draw.text(
                    (k * 800 + 15, 615),
                    f"Lip Z={s['lip_trajectory'][i, 2]:.3f} m | pile top={spatial[k]['pile_top_m'][i]:.3f} m",
                    font=font,
                    fill="white",
                )
                clearance = spatial[k]["bucket_ground_clearance_m"][i]
                draw.text(
                    (k * 800 + 15, 645),
                    f"Floor clearance={clearance * 1000:+.1f} mm | in-bucket={s['accounting'][i, col]:.1f} kg",
                    font=font,
                    fill="tomato" if clearance < 0 else "white",
                )
                gap = spatial[k]["lip_minus_particle_surface_m"][i]
                text = (
                    f"Lip above local particle surface: {gap:+.3f} m"
                    if np.isfinite(gap)
                    else "No particle column below sampled cutting edge"
                )
                draw.text((k * 800 + 15, 705), text, font=font, fill="white")
            proc.stdin.write(np.asarray(frame).tobytes())
    finally:
        renderer.close()
        proc.stdin.close()
    if proc.wait():
        raise RuntimeError("Replay encoding failed")
    return output


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--root", type=Path, default=Path("outputs/wa320/sensitivity_investigation")
    )
    print(render(p.parse_args().root).resolve())
