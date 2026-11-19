"""Scientific geometry plots and explicitly prescribed kinematic replay."""

import json
import subprocess

import numpy as np


def render_artifacts(
    output, config, model, observed, report, *, source_image=None, video=False
):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=100)
    fig.subplots_adjust(left=0.07, right=0.69, bottom=0.12, top=0.83)
    ax.set(
        xlim=(-3.7, 2.8),
        ylim=(-0.3, 4.9),
        xlabel="X forward [m] — origin at front axle",
        ylabel="Z above ground [m]",
    )
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.axhline(0, color="#555555")
    radius = config["published"]["wheel_radius_m"]
    for x in (0, -config["calibration"]["wheelbase_m"]):
        ax.add_patch(Circle((x, radius), radius, color="#70757a", alpha=0.25))
    ax.text(-3.3, 0.2, "Wheels: scale context only", fontsize=9)
    for i, pose in enumerate(observed):
        for key, point in pose.items():
            ax.scatter(*point, marker="x", color="#6a7078", s=40, zorder=6)
            ax.annotate(
                f"{key}{i}",
                point,
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=8,
            )
    ax.axhline(
        config["published"]["raised_bucket_pin_height_m"],
        color="#5b8a72",
        linestyle=":",
    )
    ax.text(-3.5, 4.07, "Published raised bucket pin: 4.005 m", fontsize=9)
    (line_boom,) = ax.plot([], [], color="#d19500", lw=5)
    (line_crank,) = ax.plot([], [], color="#276ba0", lw=4)
    (line_cylinder,) = ax.plot([], [], color="#a76628", lw=3, linestyle="--")
    dots = ax.scatter([], [], s=40, color="#222222", zorder=7)
    labels = [
        ax.text(0, 0, key, fontsize=11, fontweight="bold")
        for key in ("B", "P", "C", "E", "D?", "T?")
    ]
    fig.text(
        0.055,
        0.94,
        "WA320-7 | Brochure-derived boom and bellcrank",
        fontsize=19,
        weight="bold",
    )
    fig.text(
        0.055,
        0.895,
        "Prescribed kinematic replay — NOT a dynamics or hydraulic-force simulation",
        fontsize=12,
        color="#a34116",
    )
    side = fig.text(0.72, 0.79, "", fontsize=11, va="top", family="monospace")
    fig.text(
        0.055,
        0.035,
        "Source: Komatsu CJA00622-06, p16 | Crosses: manual drawing picks | D/T provisional | Full bucket loop unresolved",
        fontsize=9,
    )
    with np.load(output / "kinematics.npz") as saved:
        times, states, fractions = (
            saved["time_s"],
            saved["points_XZ_m"],
            saved["fraction"],
        )
    telemetry = []

    def update(i):
        points = states[i]
        _b, p, _c, e, _d, t = points
        line_boom.set_data(points[[0, 2, 1, 0]].T)
        line_crank.set_data(points[[2, 3, 4, 2]].T)
        line_cylinder.set_data(points[[5, 3]].T)
        dots.set_offsets(points)
        for label, xy in zip(labels, points):
            label.set_position(xy + [0.04, 0.04])
        length = float(np.linalg.norm(e - t))
        side.set_text(
            f"Recorded t: {times[i]:5.2f} s\nLift fraction: {fractions[i]:.3f}\n\n"
            f"Boom length: {report['boom_length_m']:.3f} m\n"
            f"C offset: {model.crank_local[1]:.3f} m\n"
            f"Crank angle: {report['crank_included_angle_deg']:.1f} deg\n\n"
            f"Bucket hinge height:\n  {p[1]:.3f} m\n"
            f"T?-E pin distance:\n  {length:.3f} m\n  (unverified, NOT stroke)\n\n"
            "B: fitted boom base\nP: bucket hinge\nC: crank pivot\nE: tilt-cylinder pin\nD?: rod pin candidate\nT?: fixed-pin candidate\n\n"
            "No bucket/rod closure\nNo mass/force claims\nActive model unchanged"
        )
        telemetry.append(
            {
                "time_s": float(times[i]),
                "tilt_pin_distance_m": length,
                "points_XZ_m": points.tolist(),
            }
        )

    update(0)
    fig.savefig(output / "reconstructed_boom.png")
    if video:
        telemetry.clear()
        command = [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgba",
            "-s",
            "1280x720",
            "-r",
            "25",
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output / "brochure_boom_kinematic_replay.mp4"),
        ]
        with subprocess.Popen(command, stdin=subprocess.PIPE) as encoder:
            for i in range(len(times)):
                update(i)
                fig.canvas.draw()
                encoder.stdin.write(fig.canvas.buffer_rgba())
            encoder.stdin.close()
            if encoder.wait():
                raise RuntimeError("Video encoding failed")
    plt.close(fig)
    (output / "kinematic_telemetry.json").write_text(
        json.dumps(telemetry, indent=2) + "\n"
    )
    if source_image:
        from PIL import Image, ImageDraw, ImageFont

        with Image.open(source_image) as image:
            if list(image.size) != config["source"]["raster_size_px"]:
                raise ValueError("Source raster must match digitization dimensions")
            crop = image.crop((1900, 1570, 2430, 2250)).resize((1060, 1360))
        draw = ImageDraw.Draw(crop)
        font = ImageFont.truetype("DejaVuSans.ttf", 22)
        for name, pose in config["picks"].items():
            if name not in ("lowered", "raised"):
                continue
            names = {
                "bucket_pin_px": "P",
                "crank_pivot_px": "C",
                "tilt_pin_px": "E",
                "rod_pin_px": "D?",
            }
            for key, text in names.items():
                if key not in pose:
                    continue
                x, y = (np.array(pose[key]) - [1900, 1570]) * 2
                color = "#b03a12" if text.endswith("?") else "#0055cc"
                draw.ellipse((x - 7, y - 7, x + 7, y + 7), outline=color, width=3)
                draw.text(
                    (x + 10, y - 25),
                    text,
                    fill=color,
                    font=font,
                    stroke_width=1,
                    stroke_fill="white",
                )
        for name, text in (("boom_base_px", "B?"), ("tilt_base_px", "T?")):
            x, y = (np.array(config["picks"][name]) - [1900, 1570]) * 2
            draw.ellipse((x - 7, y - 7, x + 7, y + 7), outline="#b03a12", width=3)
            draw.text(
                (x - 50, y - 35 if name == "tilt_base_px" else y + 15),
                text,
                fill="#b03a12",
                font=font,
                stroke_width=1,
                stroke_fill="white",
            )
        for i, pixel in enumerate(
            config["picks"]["raised"]["bucket_anchor_hypotheses_px"]
        ):
            x, y = (np.array(pixel) - [1900, 1570]) * 2
            label_x, label_y = 80 + i * 450, 50
            draw.ellipse((x - 8, y - 8, x + 8, y + 8), outline="#b03a12", width=3)
            draw.line((x, y, label_x + 50, label_y + 30), fill="#b03a12", width=2)
            draw.text(
                (label_x, label_y),
                f"F? hypothesis {i + 1}",
                fill="#b03a12",
                font=font,
                stroke_width=1,
                stroke_fill="white",
            )
        crop.save(output / "local_annotated_brochure_crop.png")
