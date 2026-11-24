"""Plot and replay recorded M2 states; never advance or correct physics."""

import argparse
import json
import os
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from wheel_loader.environment.simulation.surface_visual import (
    apply_surface_texture,
    surface_xml,
)
from wheel_loader.environment.simulation.tire_visual import replay_xml_with_tires
from wheel_loader.environment.simulation.vehicle_visual import apply_loader_palette
from wheel_loader.evaluation.material_accounting import bucket_payload_mask
from wheel_loader.vehicle.parameters import load_parameters


def artifacts(folder):
    report = json.loads((folder / "report.json").read_text())
    with np.load(folder / "recording.npz") as n:
        d = {k: n[k] for k in n.files}
    suite_path = folder.parent / "suite.json"
    suite = json.loads(suite_path.read_text()) if suite_path.exists() else None
    t = d["time_s"]
    columns = list(d["accounting_columns"])
    acc = d["accounting"]
    funnel = report.get("acquisition_funnel")
    if funnel:
        stages = [
            "initial_material",
            "contacted_by_bucket",
            "entered_mouth",
            "retained_after_breakout",
            "spilled_during_extraction_unique",
            "final_payload",
        ]
        labels = [
            "Initial material",
            "Bucket contact",
            "Mouth entry",
            "Retained at breakout",
            "Spilled during extraction",
            "Final payload",
        ]
        fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)
        for ax, field, unit in zip(
            axes, ("count", "mass_kg"), ("Particle count", "Material mass [kg]")
        ):
            values = [funnel[s][field] for s in stages]
            ax.barh(labels, values)
            ax.set_xlabel(unit)
            for i, value in enumerate(values):
                ax.text(
                    value,
                    i,
                    f" {value:.1f}" if field == "mass_kg" else f" {value}",
                    va="center",
                )
            ax.margins(x=0.18)
        axes[0].invert_yaxis()
        fig.suptitle(
            "Acquisition funnel — unique IDs; spillage is a subset of acquired material"
        )
        fig.tight_layout()
        fig.savefig(folder / "acquisition_funnel.png", dpi=150)
        plt.close(fig)
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    for name in (
        "remaining_mass_kg",
        "bucket_mass_kg",
        "spilled_mass_kg",
        "boundary_loss_mass_kg",
    ):
        axes[0].plot(t, acc[:, columns.index(name)], label=name.replace("_mass_kg", ""))
    axes[0].set_ylabel("Material mass [kg]")
    axes[0].legend(ncol=2)
    for i, name in enumerate(("steer", "lift", "bucket")):
        axes[1].plot(t, d["cylinder_force_N"][:, i] / 1000, label=name)
    axes[1].set_ylabel("Cylinder force [kN]")
    axes[1].legend()
    for i in range(4):
        axes[2].plot(t, d["wheel_slip_m_s"][:, i], label=f"wheel {i}")
    axes[2].set_ylabel("Slip velocity [m/s]")
    axes[2].legend(ncol=4)
    axes[3].plot(t, acc[:, columns.index("conservation_residual_kg")])
    axes[3].set_ylabel("Mass residual [kg]")
    axes[3].set_xlabel("Simulation time [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("M2 coarse-grain CPU experiment — uncalibrated contact/material")
    fig.tight_layout()
    fig.savefig(folder / "metrics.png", dpi=150)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(
        t, acc[:, columns.index("bucket_mass_kg")], label="instantaneous bucket mass"
    )
    ax.plot(
        t,
        acc[:, columns.index("bucket_exit_mass_kg")],
        label="acquired IDs currently outside bucket",
    )
    ax.set(
        xlabel="Simulation time [s]",
        ylabel="Material mass [kg]",
        title="Measured payload and bucket exits — centroid classification",
    )
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(folder / "payload_vs_time.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharex=True, sharey=True)
    for ax, key in zip(axes, ("initial_pile_xyz", "final_pile_xyz")):
        points = d[key]
        ax.scatter(points[:, 0], points[:, 2], s=12, c=points[:, 1], cmap="cividis")
        ax.set_title(key.replace("_", " "))
        ax.set_xlabel("World X [m]")
        ax.set_ylabel("World Z [m]")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(folder / "pile_before_after.png", dpi=150)
    plt.close(fig)
    model = mujoco.MjModel.from_xml_string(
        surface_xml(replay_xml_with_tires(folder / "model.xml"), "offroad")
    )
    apply_surface_texture(model, "offroad")
    data = mujoco.MjData(model)
    model.vis.global_.offwidth = 1600
    model.vis.global_.offheight = 900
    for g in range(model.ngeom):
        b = int(model.geom_bodyid[g])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        if name.endswith("_wheel"):
            continue
        if b == 0 and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            model.geom_size[g, :2] = [
                25,
                25,
            ]  # Render extent only; physics plane is infinite.
            continue
    apply_loader_palette(model)
    model.vis.headlight.ambient[:] = [0.65] * 3
    model.vis.headlight.diffuse[:] = [0.7] * 3
    camera = mujoco.MjvCamera()
    camera.distance = 14
    camera.azimuth = 120
    camera.elevation = -23
    vehicle = load_parameters("configs/vehicle/wa320.yaml")
    bucket_body = next(
        b
        for b in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "").endswith(
            "_bucket"
        )
    )
    grain_geoms = [
        next(g for g in range(model.ngeom) if model.geom_bodyid[g] == b)
        for b in d["grain_body_ids"]
    ]
    initial = d["initial_pile_xyz"]
    camera.lookat[:] = [1.5 + initial[:, 0].min() / 2, 0, 1.6]
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 23)
    output = folder / "scripted_scoop_replay.mp4"
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
            "1600x900",
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
        with mujoco.Renderer(model, height=900, width=1600) as renderer:
            for i, q in enumerate(d["qpos"]):
                data.qpos[:] = q
                mujoco.mj_forward(model, data)
                local = (
                    data.xpos[d["grain_body_ids"]] - data.xpos[bucket_body]
                ) @ data.xmat[bucket_body].reshape(3, 3)
                payload_mask = bucket_payload_mask(local, vehicle)
                measured_mass = float(d["masses_kg"][payload_mask].sum())
                if not np.isclose(
                    measured_mass,
                    acc[i, columns.index("bucket_mass_kg")],
                    rtol=1e-12,
                    atol=1e-9,
                ):
                    raise ValueError(
                        "Visual payload classifier disagrees with recorded accounting"
                    )

                for geom, loaded in zip(grain_geoms, payload_mask):
                    model.geom_rgba[geom] = (
                        [0.15, 0.85, 0.4, 1] if loaded else [0.65, 0.49, 0.28, 1]
                    )
                renderer.update_scene(data, camera)
                frame = Image.fromarray(renderer.render())
                draw = ImageDraw.Draw(frame)
                draw.rectangle((0, 0, 1600, 150), fill=(19, 24, 30))
                payload = acc[i, columns.index("bucket_mass_kg")]
                spilled = acc[i, columns.index("bucket_exit_mass_kg")]
                exited_ids = {
                    event["id"]
                    for event in (funnel or {}).get("mouth_crossing_events", [])
                    if event["event"] == "mouth_exit_after_acquisition"
                    and event["time_s"] is not None
                    and event["time_s"] <= t[i]
                }
                verified_exits = (
                    float(d["masses_kg"][list(exited_ids)].sum()) if exited_ids else 0.0
                )
                draw.text(
                    (20, 10),
                    f"M2 recorded CPU physics | {t[i]:.1f}s | {d['phase'][i]} | {report['element_count']} free grains",
                    font=font,
                    fill="white",
                )
                draw.text(
                    (20, 43),
                    f"Bucket {payload:.1f} kg | outside classifier {spilled:.1f} kg | verified mouth exits {verified_exits:.1f} kg",
                    font=font,
                    fill="white",
                )
                status = (
                    "single-run checks passed"
                    if report["single_run_passed"]
                    else "ACCEPTANCE INCOMPLETE: see report.json"
                )
                if not report.get("m2_complete", False):
                    status += " | M2 INCOMPLETE"
                if suite is not None and not suite["m2_cpu_reference_passed"]:
                    status = (
                        "Single-run checks pass"
                        if report["single_run_passed"]
                        else "THIS RUN FAILED"
                    ) + " | M2 SUITE INCOMPLETE: see suite.json"
                draw.text(
                    (20, 76),
                    status,
                    font=font,
                    fill="white",
                )
                draw.text(
                    (20, 110),
                    "Uncalibrated coarse grains | green = payload | rigid floor | replay only",
                    font=font,
                    fill="white",
                )
                proc.stdin.write(np.asarray(frame).tobytes())
            camera.distance = 5.0
            camera.azimuth = 195
            camera.elevation = -25
            camera.lookat[:] = data.xpos[bucket_body] + data.xmat[bucket_body].reshape(
                3, 3
            ) @ [0.6, 0, 0.2]
            renderer.update_scene(data, camera)
            frame = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 1600, 55), fill=(19, 24, 30))
            draw.text(
                (20, 15),
                f"Recorded retained payload {payload:.1f} kg | coarse uncalibrated material | green = payload centers",
                font=font,
                fill="white",
            )
            frame.save(folder / "bucket_payload.png")
    finally:
        proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError("Replay encoding failed")

    def point_cloud(name, positions, mask):
        ids = np.flatnonzero(mask)
        header = (
            "ply\nformat ascii 1.0\nelement vertex "
            + str(len(ids))
            + "\nproperty float x\nproperty float y\nproperty float z\nproperty float radius\nproperty float mass_kg\nproperty int material_id\nend_header\n"
        )
        rows = [
            " ".join(map(str, [*positions[i], d["radii_m"][i], d["masses_kg"][i], i]))
            for i in ids
        ]
        (folder / name).write_text(header + "\n".join(rows) + "\n")

    point_cloud(
        "initial_pile.ply",
        d["initial_pile_xyz"],
        np.ones(len(payload_mask), dtype=bool),
    )
    point_cloud("final_ground_material.ply", d["final_pile_xyz"], ~payload_mask)
    point_cloud("final_bucket_payload.ply", d["final_pile_xyz"], payload_mask)
    print(output.resolve())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    artifacts(args.folder)
