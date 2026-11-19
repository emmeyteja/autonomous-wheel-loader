"""Render recorded five-angle chassis checks; does not step or correct physics."""

import argparse
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.input / "chassis_zero/model.xml"))
    data = mujoco.MjData(model)
    model.vis.global_.offwidth = 1920
    model.vis.global_.offheight = 1080
    for g in range(model.ngeom):
        name = (
            mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])
            )
            or ""
        )
        model.geom_rgba[g] = (
            [0.3, 0.3, 0.3, 1]
            if model.geom_bodyid[g] == 0
            else [0.10, 0.11, 0.12, 1]
            if name.endswith("_wheel")
            else [0.35, 0.38, 0.41, 1]
            if name.endswith("_rod") or "hinge_housing" in name
            else [0.16, 0.32, 0.4, 1]
            if name.endswith("_cab")
            else [0.95, 0.61, 0.04, 1]
            if name
            else [0.3, 0.3, 0.3, 1]
        )
    rear = next(
        i
        for i in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").endswith(
            "_rear_frame"
        )
    )
    model.vis.headlight.ambient[:] = [0.65, 0.65, 0.65]
    model.vis.headlight.diffuse[:] = [0.7, 0.7, 0.7]
    camera = mujoco.MjvCamera()
    camera.distance = 7.4
    camera.elevation = -18
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 28)
    command = [
        "ffmpeg",
        "-y",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        "1920x1080",
        "-framerate",
        "20",
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(args.output),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    try:
        with mujoco.Renderer(model, height=1080, width=1920) as renderer:
            for case, angle in [
                ("chassis_zero", 0),
                ("chassis_pos20", 20),
                ("chassis_neg20", -20),
                ("chassis_poslimit", 38.5),
                ("chassis_neglimit", -38.5),
            ]:
                with np.load(args.input / case / "dynamics.npz") as recording:
                    q = recording["qpos"][-60:]
                for index, row in enumerate(q):
                    data.qpos[:] = row
                    mujoco.mj_forward(model, data)
                    camera.lookat[:] = data.xpos[rear] + [0.7, 0, 0.15]
                    camera.azimuth = 85 + 10 * index / max(1, len(q) - 1)
                    renderer.update_scene(data, camera)
                    frame = Image.fromarray(renderer.render())
                    draw = ImageDraw.Draw(frame)
                    draw.rectangle((0, 0, 1920, 150), fill=(20, 24, 30))
                    draw.text(
                        (25, 15),
                        f"WA320 v1 physical hinge close-up | articulation {angle:+g} deg",
                        font=font,
                        fill="white",
                    )
                    draw.text(
                        (25, 60),
                        "Recorded settled physics; no state correction | estimated frame/mount dimensions",
                        font=font,
                        fill="white",
                    )
                    draw.text(
                        (25, 105),
                        "Upper/lower pins, bearing housings and cylinders carry physical mass and collisions",
                        font=font,
                        fill="white",
                    )
                    process.stdin.write(np.asarray(frame).tobytes())
    finally:
        process.stdin.close()
    if process.wait() != 0:
        raise RuntimeError("Video encoding failed")
    print(args.output.resolve())
