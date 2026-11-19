"""Bucket registration figures and explicitly prescribed, recorded kinematics."""

import subprocess

import numpy as np

from wheel_loader.vehicle.brochure_geometry import rotation


def render_bucket_artifacts(
    output, base, config, poses, report, *, source_image=None, video=False
):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon

    with np.load(output / "bucket_kinematics.npz") as saved:
        times = saved["time_s"]
        points = saved["points_XZ_m"]
        angles = saved["bucket_angles_rad"]
        profiles = saved["bucket_profile_XZ_m"]
        anchors = saved["anchor_world_XZ_m"]
    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=100)
    fig.subplots_adjust(left=0.065, right=0.68, bottom=0.12, top=0.82)
    ax.set(
        xlim=(-3.8, 3.9),
        ylim=(-0.4, 6),
        xlabel="X forward [m] — origin at front axle",
        ylabel="Z above ground [m]",
    )
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.axhline(0, color="#555555")
    for x in (0, -3.03):
        ax.add_patch(Circle((x, 0.695), 0.695, color="#737b82", alpha=0.2))
    ax.text(-3.5, 0.18, "Wheel context only", fontsize=9)
    (boom_line,) = ax.plot([], [], color="#c58b00", lw=4)
    (crank_line,) = ax.plot([], [], color="#326890", lw=3)
    bucket = Polygon(
        profiles[0], closed=True, facecolor="#edc449", edgecolor="none", alpha=0.2
    )
    ax.add_patch(bucket)
    (shell_line,) = ax.plot([], [], color="#b07a00", lw=3)
    (mouth_line,) = ax.plot([], [], color="#b07a00", lw=1.5, linestyle=":")
    pins = ax.scatter([], [], s=40, color="#1b5688", zorder=7)
    pin_texts = [ax.text(0, 0, s, fontsize=10) for s in ("P", "F?", "L", "M")]
    for name, pose in poses.items():
        color = {
            "ground": "#8a5b20",
            "raised_rollback": "#456aab",
            "raised_dump": "#3b8c69",
        }[name]
        for key in ("lip", "mouth_top", "anchor"):
            ax.scatter(
                *pose[key],
                color=color,
                marker="x",
                s=28,
                alpha=0.7,
                label=name.replace("_", " ") if key == "lip" else None,
            )
    ax.legend(loc="upper left", fontsize=8, title="Drawing picks", title_fontsize=8)
    ax.axhline(
        base["published"]["dump_clearance_m"], color="#3b8c69", linestyle=":", alpha=0.6
    )
    ax.text(-3.5, 2.9, "Published dump clearance 2.835 m", fontsize=8, color="#3b8c69")
    fig.text(
        0.05,
        0.945,
        "WA320-7 | Bucket outline and prescribed curl",
        fontsize=20,
        weight="bold",
    )
    fig.text(
        0.05,
        0.895,
        "KINEMATIC REPLAY ONLY — no passive rod closure, hydraulics, forces or material",
        fontsize=11,
        color="#a63b1c",
    )
    panel = fig.text(0.715, 0.8, "", va="top", family="monospace", fontsize=10.5)
    fig.text(
        0.05,
        0.035,
        "Source: Komatsu CJA00622-06, p16 | Crosses: drawing picks | Shading/closing mouth line are schematic; not collision geometry",
        fontsize=8.5,
    )

    def update(i):
        b, p, c, e = points[i]
        boom_line.set_data(np.array([b, c, p, b]).T)
        crank_line.set_data(np.array([c, e]).T)
        bucket.set_xy(profiles[i])
        shell_line.set_data(profiles[i].T)
        mouth_line.set_data(profiles[i][[-1, 0]].T)
        locations = [p, anchors[i], profiles[i][0], profiles[i][-1]]
        pins.set_offsets(locations)
        for text, xy in zip(pin_texts, locations):
            text.set_position(xy + [0.04, 0.04])
        t = times[i]
        phase = (
            "GROUND"
            if t < 2
            else "RAISE / ROLLBACK"
            if t < 6
            else "RAISED HOLD"
            if t < 8
            else "DUMP CURL"
            if t < 10
            else "DUMP HOLD"
            if t < 12
            else "RETURN"
        )
        panel.set_text(
            f"{phase} | t={t:.2f} s\n\nBucket reference angle:\n  {np.rad2deg(angles[i]):+.1f} deg (prescribed)\n"
            f"BOC lip X/Z:\n  {profiles[i][0, 0]:.3f} / {profiles[i][0, 1]:.3f} m\n\n"
            "P: bucket hinge\nF?: attachment candidate\nL: bolt-on cutting lip\nM: mouth top\n\n"
            "Ground + rollback fit\nDump pose held out\n(same brochure)\n\n"
            f"Dump lip residual:\n  {report['pose_residuals_m']['raised_dump']['lip'] * 1000:.0f} mm\n"
            f"Dump anchor residual:\n  {report['pose_residuals_m']['raised_dump']['anchor'] * 1000:.0f} mm\n\n"
            f"Preferred pin: {report['preferred_anchor_interpretation']}\nPrior D/one-rod hypothesis:\n  {'CONSISTENT' if report['rod_hypothesis']['fixed_D_single_rod_consistent'] else 'NOT CONSISTENT'}\nNo unique linkage identified\n\n"
            "Active vehicle unchanged\nGround lip fit below reference\nNo contact/force validation"
        )

    update(0)
    fig.savefig(output / "bucket_reconstruction.png")
    if video:
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
            str(output / "bucket_lift_curl_kinematic_replay.mp4"),
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
    if source_image:
        from PIL import Image, ImageDraw, ImageFont

        with Image.open(source_image) as source:
            if list(source.size) != base["source"]["raster_size_px"]:
                raise ValueError("Incorrect source raster dimensions")
            image = source.crop((1810, 1440, 2280, 2260)).resize((940, 1640))
        draw = ImageDraw.Draw(image)
        font = ImageFont.truetype("DejaVuSans.ttf", 20)
        scale = base["calibration"]["wheelbase_m"] / (
            base["calibration"]["rear_axle_px"][0]
            - base["calibration"]["front_axle_px"][0]
        )
        origin = np.array(
            [
                base["calibration"]["front_axle_px"][0],
                base["calibration"]["ground_y_px"],
            ]
        )
        profile = np.array(report["profile_vertices_m"])
        for name, pose in poses.items():
            color = {
                "ground": "#b26b00",
                "raised_rollback": "#276ac0",
                "raised_dump": "#168442",
            }[name]
            world = pose["P"] + profile @ rotation(pose["angle"]).T
            pixels = (origin - world / scale - [1810, 1440]) * 2
            draw.line([tuple(p) for p in pixels], fill=color, width=3)
            f = (
                origin
                - (
                    pose["P"]
                    + rotation(pose["angle"]) @ report["local_features_m"]["anchor"]
                )
                / scale
            )
            x, y = (f - [1810, 1440]) * 2
            draw.ellipse((x - 7, y - 7, x + 7, y + 7), outline=color, width=3)
            draw.text(
                (x + 10, y - 20),
                "F?",
                fill=color,
                font=font,
                stroke_width=1,
                stroke_fill="white",
            )
        image.save(output / "local_bucket_registration_overlay.png")
