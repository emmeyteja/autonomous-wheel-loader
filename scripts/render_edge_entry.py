"""Close-up replay of recorded finite-force bucket entry; no simulation stepping."""

import argparse
import json
import os
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def render(root, output, cases=None):
    output.parent.mkdir(parents=True, exist_ok=True)
    font = ImageFont.truetype("DejaVuSans.ttf", 22)
    small = ImageFont.truetype("DejaVuSans.ttf", 19)
    encoder = subprocess.Popen(
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
            "1400x800",
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
        folders = cases or [
            root / name
            for name in ["row_3_straight", "row_3_curl_040", "row_3_curl_095"]
        ]
        for folder in folders:
            name = folder.name
            report = json.loads((folder / "report.json").read_text())
            state = np.load(folder / "states.npz")
            model = mujoco.MjModel.from_binary_path(str(folder / "compiled_model.mjb"))
            model.vis.global_.offwidth, model.vis.global_.offheight = 1000, 650
            model.vis.headlight.ambient[:] = 0.7
            bucket = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "test_bucket")
            model.geom_rgba[model.geom_bodyid == bucket] = [0.22, 0.27, 0.30, 0.35]
            model.geom_rgba[0] = [0.25, 0.25, 0.25, 1]
            model.geom_size[0, :2] = 100
            data = mujoco.MjData(model)
            camera = mujoco.MjvCamera()
            camera.distance, camera.azimuth, camera.elevation = 4.0, 110, -16
            renderer = mujoco.Renderer(model, height=650, width=1000)
            try:
                for t in np.arange(0, state["time_s"][-1], 0.05):
                    i = min(
                        np.searchsorted(state["time_s"], t), len(state["time_s"]) - 1
                    )
                    data.qpos[:] = state["qpos"][i]
                    mujoco.mj_forward(model, data)
                    camera.lookat[:] = [state["bucket_pos"][i, 0] + 0.8, 0, 0.35]
                    renderer.update_scene(data, camera)
                    contacts = state["contacts"]
                    near = contacts[abs(contacts[:, 0] - state["time_s"][i]) < 1e-7]
                    frame = Image.new("RGB", (1400, 800), (20, 24, 30))
                    frame.paste(Image.fromarray(renderer.render()), (0, 95))
                    draw = ImageDraw.Draw(frame)
                    draw.rectangle((690, 525, 995, 737), fill=(25, 30, 38))
                    draw.text(
                        (705, 535), "Normal: X right / Z up", font=small, fill="cyan"
                    )
                    if len(near):
                        c = near[len(near) // 2]
                        base = np.array([830.0, 660.0])
                        direction = np.array([c[5], -c[7]])
                        direction /= max(np.linalg.norm(direction), 1e-12)
                        tip = base + 60 * direction
                        side = np.array([-direction[1], direction[0]])
                        draw.line([tuple(base), tuple(tip)], fill="cyan", width=5)
                        draw.polygon(
                            [
                                tuple(tip),
                                tuple(tip - 18 * direction + 8 * side),
                                tuple(tip - 18 * direction - 8 * side),
                            ],
                            fill="cyan",
                        )
                        draw.text(
                            (705, 568),
                            f"Elevation: {np.rad2deg(np.arcsin(np.clip(c[7], -1, 1))):+.2f} deg",
                            font=small,
                            fill="white",
                        )
                    else:
                        draw.text(
                            (705, 640),
                            "No active bucket contact",
                            font=small,
                            fill="white",
                        )
                    draw.text(
                        (18, 12),
                        f"CONTACT REPLAY | {report.get('edge_variant', 'blunt')} | h={report.get('attack_height_m', 0.085):.3f} m | {'BACKSTOP' if report.get('backstop') else 'FREE'} | {name}",
                        font=font,
                        fill="orange",
                    )
                    draw.text(
                        (18, 48),
                        "Finite-force carriage; vehicle omitted. Material unchanged. Taper variants are hypothetical. Not an M2 scoop.",
                        font=small,
                        fill="white",
                    )
                    lines = [
                        f"Time: {t:.2f} s",
                        f"{report['particle_count']} free particles",
                        f"Radius: {report['particle_radius_m']:.4f} m",
                        f"Inside: {int(state['inside'][i].sum())} / {report['particle_count']}",
                        "Bucket ground gap:",
                        f" {1000 * state['bucket_ground_clearance_m'][i]:.1f} mm",
                        "Particle force X:",
                        f" {state['force_x_N'][i]:+.1f} N",
                        "Particle force Z:",
                        f" {state['force_z_N'][i]:+.1f} N",
                        "Cyan inset: normal",
                        "Transparent: bucket",
                        f"Mouth entries: {report['mouth_entry_events']}",
                        "Tracking gate: "
                        + (
                            "PASS"
                            if max(report["maximum_carriage_tracking_error_m_m_rad"])
                            < 0.01
                            else "FAIL"
                        ),
                        "40 mm gap gate: "
                        + (
                            "PASS"
                            if report["minimum_bucket_floor_clearance_m"] >= 0.04
                            else "FAIL"
                        ),
                    ]
                    for j, line in enumerate(lines):
                        draw.text(
                            (1020, 120 + j * 39),
                            line,
                            font=font,
                            fill="tomato" if "FAIL" in line else "white",
                        )
                    draw.text(
                        (18, 758),
                        "Recorded qpos replay | provisional coarse-sphere/contact diagnostic; not real-machine force validation",
                        font=small,
                        fill="white",
                    )
                    encoder.stdin.write(np.asarray(frame).tobytes())
            finally:
                renderer.close()
    finally:
        encoder.stdin.close()
    if encoder.wait():
        raise RuntimeError("Replay encoding failed")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", type=Path, default=Path("."))
    p.add_argument("--case", type=Path, action="append")
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    render(a.root, a.output, a.case)
