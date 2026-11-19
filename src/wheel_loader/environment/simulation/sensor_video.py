"""Visualize recorded motion and generated sensor streams without stepping physics."""

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np

from .replay_state import ReplayState
from .surface_visual import apply_surface_texture, surface_xml
from .tire_visual import replay_xml_with_tires


def render_sensor_video(folder, sensors, parameters):
    import mujoco
    from PIL import Image, ImageDraw, ImageFont

    folder, sensors = Path(folder), Path(sensors)
    report = json.loads((sensors / "sensor_report.json").read_text())
    if not all(report["checks"].values()):
        raise ValueError("Sensor checks must pass before rendering")
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required")
    with np.load(folder / "motion.npz") as recording:
        times, positions, roots = (
            recording[k] for k in ("time_s", "positions", "root_poses_xyzw")
        )
        names = recording["joint_names"].tolist()
    with np.load(sensors / "sensors.npz") as stream:
        t = stream["encoder_time_s"]
        odom = stream["odometry_rear_xy_yaw"]
        st = stream["scan_capture_time_s"]
        points = stream["points_odom_m"]
        force = stream["bucket_contact_force_world_N"]
        articulation = stream["articulation_rad"]
    with np.load(sensors / "privileged_evaluation.npz") as evaluation:
        truth = evaluation["rear_truth_xy_yaw"]
    model = mujoco.MjModel.from_xml_string(
        surface_xml(replay_xml_with_tires(folder / "effective_model.xml"), "offroad")
    )
    apply_surface_texture(model, "offroad")
    model.vis.global_.offwidth = 840
    model.vis.global_.offheight = 720
    model.vis.headlight.ambient[:] = [0.5, 0.5, 0.5]
    for i in range(model.ngeom):
        body = (
            mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[i])
            )
            or ""
        )
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if "Pile" in name:
            model.geom_rgba[i] = [0.58, 0.43, 0.24, 1]
        elif model.geom_bodyid[i] and "wheel" not in body:
            model.geom_rgba[i] = (
                [0.95, 0.58, 0.08, 1]
                if "frame" in body or "boom" in body
                else [0.45, 0.47, 0.50, 1]
            )
    replay = ReplayState(model, names, positions, roots, times, parameters)
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = 17, 135, -28
    font = ImageFont.truetype("DejaVuSans.ttf", 16)
    title_font = ImageFont.truetype("DejaVuSans.ttf", 21)
    map_image = Image.new("RGB", (400, 340), (24, 32, 41))
    md = ImageDraw.Draw(map_image)

    # Fixed odometry coordinates: x in [-5, 20], y in [-12.5, 12.5].
    def pixel(xy):
        return (int((xy[0] + 5) * 16), int((12.5 - xy[1]) * 13.6))

    output = sensors / "sensor_v_cycle_replay.mp4"
    renderer = mujoco.Renderer(model, height=720, width=840)
    scan_index = 0
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
                "20",
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
            indices = np.searchsorted(times, np.arange(times[0], times[-1], 0.5))
            indices = np.append(indices, len(times) - 1)
            for i in indices:
                replay.configuration(int(i))
                camera.lookat[:] = roots[i, :3] + [1, 0, 0]
                renderer.update_scene(replay.data, camera)
                frame = Image.new("RGB", (1280, 720), (14, 21, 29))
                frame.paste(Image.fromarray(renderer.render()), (0, 0))
                while scan_index < len(st) and st[scan_index] <= times[i]:
                    cloud = points[scan_index]
                    for point in cloud[np.isfinite(cloud).all(axis=1)]:
                        x, y = pixel(point[:2])
                        if 0 <= x < 400 and 0 <= y < 340:
                            z = float(np.clip(point[2] / 3, 0, 1))
                            md.point(
                                (x, y),
                                fill=(
                                    int(60 + 190 * z),
                                    int(150 + 60 * z),
                                    int(220 - 170 * z),
                                ),
                            )
                    scan_index += 1
                j = min(int(np.searchsorted(t, times[i])), len(t) - 1)
                panel = map_image.copy()
                pd = ImageDraw.Draw(panel)
                if j > 1:
                    pd.line(
                        [pixel(v) for v in truth[: j + 1 : 4]],
                        fill=(240, 110, 110),
                        width=2,
                    )
                    pd.line(
                        [pixel(v) for v in odom[: j + 1 : 4]],
                        fill=(105, 255, 150),
                        width=2,
                    )
                x, y = pixel(odom[j])
                pd.ellipse((x - 4, y - 4, x + 4, y + 4), fill="white")
                frame.paste(panel, (860, 112))
                d = ImageDraw.Draw(frame)
                d.rectangle((0, 0, 840, 78), fill=(14, 21, 29))
                d.text(
                    (18, 12),
                    "Recorded V-cycle | offline sensor replay",
                    font=title_font,
                    fill="white",
                )
                d.text(
                    (18, 43),
                    f"t = {times[i]:6.1f} s | 10x playback | physics not rerun",
                    font=font,
                    fill="#c0ccd6",
                )
                d.text((860, 15), "Observed surface map", font=title_font, fill="white")
                d.text(
                    (860, 47), "Range scans + wheel odometry", font=font, fill="#c0ccd6"
                )
                d.text(
                    (860, 72),
                    "Includes ground / vehicle; no segmentation",
                    font=font,
                    fill="#c0ccd6",
                )
                lines = [
                    "Map: x -5..20 m, y -12.5..12.5 m",
                    "Height: blue 0 m -> amber 3 m",
                    "Green: encoder odom | red: sim truth*",
                    f"Scans {scan_index}/{len(st)} | 5 Hz | 1296 rays",
                    f"Rear axle XY: {odom[j, 0]:+.2f}, {odom[j, 1]:+.2f} m",
                    f"Articulation: {np.rad2deg(articulation[j]):+.1f} deg",
                    f"Bucket contact: {np.linalg.norm(force[j]):.0f} N",
                    "Pressure / stroke / payload: unavailable",
                    "*Truth is evaluation only, not map input",
                    "Trial sensors; real accuracy unvalidated",
                ]
                for k, line in enumerate(lines):
                    d.text((860, 461 + k * 24), line, font=font, fill="#c0ccd6")
                encoder.stdin.write(frame.tobytes())
            encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError("Video encoding failed")
    finally:
        renderer.close()
    return output
