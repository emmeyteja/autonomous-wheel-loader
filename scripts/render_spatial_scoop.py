"""Replay actual spatial-controller records with geometry and load readouts."""

import argparse
import json
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


def render(folders, output):
    output.parent.mkdir(parents=True, exist_ok=True)
    font = ImageFont.truetype("DejaVuSans.ttf", 22)
    titlefont = ImageFont.truetype("DejaVuSans.ttf", 28)
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
            "1600x800",
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
        for folder in folders:
            report = json.loads((folder / "report.json").read_text())
            s = dict(np.load(folder / "states.npz"))
            control = np.load(folder / "controller.npz")["values"]
            model = mujoco.MjModel.from_xml_string(
                surface_xml(
                    replay_xml_with_tires(folder / "bundle/model.xml"), "asphalt"
                )
            )
            apply_surface_texture(model, "asphalt")
            apply_loader_palette(model)
            for g in range(model.ngeom):
                if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
                    model.geom_size[g, :2] = 100
            model.vis.global_.offwidth, model.vis.global_.offheight = 1100, 650
            model.vis.headlight.ambient[:] = 0.65
            renderer = mujoco.Renderer(model, height=650, width=1100)
            data = mujoco.MjData(model)
            camera = mujoco.MjvCamera()
            camera.distance = 11
            camera.azimuth = 100
            camera.elevation = -15
            rear = next(
                i
                for i in range(model.nbody)
                if (
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
                ).endswith("_rear_frame")
            )
            try:
                for t in np.arange(0, s["time_s"][-1], 0.05):
                    i = min(np.searchsorted(s["time_s"], t), len(s["time_s"]) - 1)
                    c = control[i]
                    data.qpos[:] = s["qpos"][i]
                    mujoco.mj_forward(model, data)
                    camera.lookat[:] = s["body_positions"][i, rear] + [1.6, 0, 0.4]
                    renderer.update_scene(data, camera)
                    frame = Image.new("RGB", (1600, 800), (20, 24, 30))
                    frame.paste(Image.fromarray(renderer.render()), (0, 100))
                    d = ImageDraw.Draw(frame)
                    label = (
                        "UNLOADED SEQUENCE COMPLETE"
                        if report["passed"] and not report["material_present"]
                        else "CONTROLLER SEQUENCE COMPLETE"
                        if report["passed"]
                        else "FAILED ACCEPTANCE: " + str(report["controller_reason"])
                    )
                    d.text(
                        (20, 12),
                        label,
                        font=titlefont,
                        fill="lightgreen" if report["passed"] else "tomato",
                    )
                    d.text(
                        (20, 52),
                        f"Recorded {report['backend']} feedback run | no pose corrections | estimated physics; not real-machine validation",
                        font=font,
                        fill="white",
                    )
                    lines = [
                        f"t={s['time_s'][i]:.2f} s",
                        f"Phase: {s['phase'][i]}",
                        f"Lip X: {c[1]:.3f} m",
                        f"Lip Z: {c[2]:.3f} m",
                        f"Cutting pitch: {np.rad2deg(c[3]):+.1f} deg",
                        f"Floor clearance: {1000 * c[4]:+.1f} mm",
                        f"Penetration: {c[5]:+.3f} m",
                        f"Material contact: {bool(c[7])}",
                        f"In bucket: {c[6]:.1f} kg",
                        f"Drive target: {c[10]:+.2f} m/s",
                        f"Lift command: {c[11]:+.3f} m/s",
                        f"Curl command: {c[12]:+.3f} m/s",
                        f"Reaction: {c[13] / 1000:.2f} kN",
                    ]
                    for j, line in enumerate(lines):
                        d.text(
                            (1115, 115 + j * 38),
                            line,
                            font=font,
                            fill="tomato" if j == 5 and c[4] < 0 else "white",
                        )
                    d.text(
                        (20, 760),
                        "Yellow vehicle / steel rods / dark bucket | ground clearance and material-entry gates active",
                        font=font,
                        fill="white",
                    )
                    proc.stdin.write(np.asarray(frame).tobytes())
            finally:
                renderer.close()
    finally:
        proc.stdin.close()
    if proc.wait():
        raise RuntimeError("Replay encoding failed")
    return output


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", action="append", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    print(render(a.run, a.output).resolve())
