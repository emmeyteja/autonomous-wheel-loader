"""Render recorded ground motion; never advances or changes simulation physics."""

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np


def render_replay(folder, output=None, *, fps=20, speedup=1.0, parameters=None):
    import mujoco
    from PIL import Image, ImageDraw

    folder = Path(folder)
    output = Path(output) if output else folder / "replay.mp4"
    if not isinstance(fps, int) or isinstance(fps, bool) or not 1 <= fps <= 240:
        raise ValueError("Video frame rate must be an integer from 1 to 240")
    if (
        not isinstance(speedup, (int, float))
        or not np.isfinite(speedup)
        or speedup <= 0
    ):
        raise ValueError("Playback speed must be finite and positive")
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("Install ffmpeg to encode simulation video")
    with np.load(folder / "motion.npz", allow_pickle=False) as recording:
        positions = recording["positions"]
        roots = recording["root_poses_xyzw"]
        times = recording["time_s"]
        names = recording["joint_names"].tolist()
        phase_ids = recording["phase_ids"] if "phase_ids" in recording else np.array([])
        phase_names = (
            recording["phase_names"].tolist() if "phase_names" in recording else []
        )
        commands = recording["commands"] if "commands" in recording else np.array([])
    if (
        len(times) < 2
        or not np.all(np.diff(times) > 0)
        or roots.shape != (len(times), 7)
        or positions.shape != (len(times), len(names))
        or any(not np.isfinite(v).all() for v in (times, positions, roots))
        or not np.allclose(np.linalg.norm(roots[:, 3:], axis=1), 1, atol=1e-4)
    ):
        raise ValueError("Invalid recorded trajectory")
    from wheel_loader.vehicle.parameters import load_parameters

    from .replay_overlay import draw_cylinders, draw_state_panel
    from .replay_state import ReplayState
    from .tire_visual import replay_xml_with_tires

    if parameters is None:
        parameters = load_parameters(
            Path(__file__).resolve().parents[4] / "configs/vehicle/wa320.yaml"
        )

    model = mujoco.MjModel.from_xml_string(
        replay_xml_with_tires(folder / "effective_model.xml")
    )
    model.vis.global_.offwidth, model.vis.global_.offheight = 1280, 1080
    model.vis.headlight.active = 1
    model.vis.headlight.ambient[:] = [0.5, 0.5, 0.5]
    model.vis.headlight.diffuse[:] = [0.8, 0.8, 0.8]
    # Display colors only: no dynamics are stepped during this replay.
    for i in range(model.ngeom):
        body = (
            mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[i])
            )
            or ""
        )
        if model.geom_bodyid[i] == 0:
            model.geom_rgba[i] = [0.35, 0.39, 0.42, 1]
            if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE:
                model.geom_size[i, :2] = [50, 50]
        elif "wheel" in body:
            pass  # Procedural tire colors and hidden contact cylinders are retained.
        elif any(part in body for part in ("bucket", "rod", "bellcrank")):
            model.geom_rgba[i] = [0.45, 0.47, 0.50, 1]
        else:
            model.geom_rgba[i] = [0.95, 0.58, 0.08, 1]
    telemetry = ReplayState(model, names, positions, roots, times, parameters)
    data = telemetry.data
    # The original box remains in the recorded physics model, but the replay
    # displays its rounded connecting-rod replacement and separate cylinders.
    model.geom_rgba[model.geom_bodyid == telemetry.bodies["rod"], 3] = 0
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = 13, 135, -22
    output.parent.mkdir(parents=True, exist_ok=True)
    renderer = mujoco.Renderer(model, height=1080, width=1280)
    try:
        with (
            subprocess.Popen(
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
                    "1920x1080",
                    "-r",
                    str(fps),
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
            ) as encoder,
            output.with_suffix(".telemetry.jsonl").open("w") as telemetry_file,
        ):
            telemetry_file.write(
                json.dumps(
                    {
                        "world_frame": "X forward at start, Y left, Z up",
                        "joint_names": names,
                        "units": {
                            "position": "m",
                            "joint_angle": "rad",
                            "linear_velocity": "m/s",
                            "angular_velocity": "rad/s",
                        },
                        "provenance": "reconstructed recorded configurations; finite differences in simulation time",
                        "hydraulics": "estimated visual mounting geometry only; pressure/flow/force/absolute stroke unknown",
                    }
                )
                + "\n"
            )
            try:
                indices = np.searchsorted(
                    times, np.arange(times[0], times[-1], speedup / fps)
                )
                for index in indices:
                    root = roots[index]
                    state = telemetry.sample(index)
                    telemetry_file.write(
                        json.dumps(
                            {
                                "time_s": float(times[index]),
                                "joint_angles_rad": positions[index],
                                **state,
                            },
                            default=lambda value: value.tolist(),
                        )
                        + "\n"
                    )
                    camera.lookat[:] = root[:3] + np.array([0.5, 0, 0.7])
                    renderer.update_scene(data, camera)
                    draw_cylinders(
                        renderer.scene,
                        state,
                        telemetry.initial_lengths,
                        data,
                        telemetry.bodies["rod"],
                        parameters["linkage"]["rod_length"],
                    )
                    frame = Image.new("RGB", (1920, 1080))
                    frame.paste(Image.fromarray(renderer.render()), (0, 0))
                    draw_state_panel(frame, state, names, positions[index])
                    if len(phase_ids) == len(times):
                        draw = ImageDraw.Draw(frame)
                        draw.rectangle((12, 12, 780, 70), fill=(15, 20, 25))
                        phase = (
                            phase_names[int(phase_ids[index])]
                            .replace("reverse_loaded", "reverse bucket raised")
                            .replace("_", " ")
                            .upper()
                        )
                        velocity = commands[index, 0]
                        gear = (
                            "REVERSE"
                            if velocity < -0.01
                            else ("FORWARD" if velocity > 0.01 else "STOPPED")
                        )
                        draw.text(
                            (24, 22),
                            f"{phase} | {gear} | t={times[index]:.1f}s | {speedup:g}x replay",
                            fill="white",
                        )
                        draw.text(
                            (24, 45),
                            "Unladen motion rehearsal - no sand acquisition or payload transfer",
                            fill=(255, 205, 90),
                        )
                        draw.rectangle((1020, 12, 1268, 256), fill=(15, 20, 25))
                        low, high = (
                            roots[:, :2].min(axis=0) - 1,
                            roots[:, :2].max(axis=0) + 1,
                        )

                        def map_xy(point, low=low, high=high):
                            value = (point - low) / (high - low)
                            return (
                                1040 + int(208 * value[0]),
                                232 - int(190 * value[1]),
                            )

                        route = [map_xy(point) for point in roots[::120, :2]]
                        draw.line(route, fill=(100, 110, 120), width=2)
                        past = [map_xy(point) for point in roots[: index + 1 : 120, :2]]
                        past.append(map_xy(root[:2]))
                        if len(past) > 1:
                            draw.line(past, fill=(255, 170, 40), width=3)
                        px, py = map_xy(root[:2])
                        draw.ellipse((px - 4, py - 4, px + 4, py + 4), fill="white")
                        ox, oy = map_xy(np.array([0.0, 0.0]))
                        draw.ellipse(
                            (ox - 4, oy - 4, ox + 4, oy + 4),
                            outline=(80, 255, 120),
                            width=2,
                        )
                        draw.text(
                            (1034, 24), "Recorded route | green: origin", fill="white"
                        )
                    encoder.stdin.write(np.asarray(frame).tobytes())
            finally:
                encoder.stdin.close()
            if encoder.wait():
                raise RuntimeError("Video encoding failed")
    finally:
        renderer.close()
    return output.resolve()
