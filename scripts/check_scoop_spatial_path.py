"""Offline spatial audit of recorded bucket collision geometry and particle XYZ."""

import argparse
import itertools
import json
from pathlib import Path

import mujoco
import numpy as np

from wheel_loader.environment.simulation.backend_benchmark import load_bundle
from wheel_loader.vehicle.parameters import load_parameters


def audit(bundle, states_file, output):
    model, _, manifest = load_bundle(bundle)
    source_report = json.loads((states_file.parent / "report.json").read_text())
    if source_report["bundle_sha256"] != manifest["sha256"]:
        raise ValueError("Recorded states use a different model/input bundle")
    if (
        model.geom_type[0] != mujoco.mjtGeom.mjGEOM_PLANE
        or not np.array_equal(model.geom_pos[0], np.zeros(3))
        or not np.array_equal(model.geom_quat[0], [1, 0, 0, 0])
    ):
        raise ValueError("This audit requires the preserved z=0 world plane")
    states = dict(np.load(states_file))
    parameters = load_parameters("configs/vehicle/wa320.yaml")
    bucket = next(
        b
        for b in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "").endswith(
            "_bucket"
        )
    )
    local_vertices = []
    for g in np.flatnonzero(model.geom_bodyid == bucket):
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            vertices = (
                np.array(list(itertools.product([-1, 1], repeat=3)))
                * model.geom_size[g]
            )
        elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[g]
            start = model.mesh_vertadr[mesh]
            count = model.mesh_vertnum[mesh]
            vertices = model.mesh_vert[start : start + count]
        else:
            raise ValueError("Unsupported bucket collider in spatial audit")
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, model.geom_quat[g])
        local_vertices.extend(vertices @ rotation.reshape(3, 3).T + model.geom_pos[g])
    rotation = states["body_rotations"][:, bucket]
    position = states["body_positions"][:, bucket]
    world = (
        np.einsum("nij,vj->nvi", rotation, np.asarray(local_vertices))
        + position[:, None, :]
    )
    clearance = world[:, :, 2].min(axis=1)
    grain_ids = states["grain_body_ids"]
    radii = np.array(
        [
            model.geom_size[np.flatnonzero(model.geom_bodyid == b)[0], 0]
            for b in grain_ids
        ]
    )
    grain_xyz = states["body_positions"][:, grain_ids]
    lip = np.asarray(parameters["bucket"]["profile"])[0]
    width = parameters["vehicle"]["bucket_width"]
    lip_local = np.array(
        [[lip[0], y, lip[1]] for y in np.linspace(-width / 2, width / 2, 21)]
    )
    lip_world = np.einsum("nij,vj->nvi", rotation, lip_local) + position[:, None, :]
    delta = lip_world[:, :, None, :2] - grain_xyz[:, None, :, :2]
    radial = radii[None, None, :] ** 2 - np.sum(delta**2, axis=-1)
    top = np.where(
        radial >= 0, grain_xyz[:, None, :, 2] + np.sqrt(np.maximum(radial, 0)), -np.inf
    ).max(axis=2)
    gap = np.where(np.isfinite(top), lip_world[:, :, 2] - top, np.inf).min(axis=1)
    phases = {}
    for phase in dict.fromkeys(states["phase"]):
        mask = states["phase"] == phase
        valid = mask & np.isfinite(gap)
        phases[str(phase)] = {
            "start_s": float(states["time_s"][mask][0]),
            "end_s": float(states["time_s"][mask][-1]),
            "min_bucket_collider_ground_clearance_m": float(clearance[mask].min()),
            "lip_center_z_min_max_m": [
                float(states["lip_trajectory"][mask, 2].min()),
                float(states["lip_trajectory"][mask, 2].max()),
            ],
            "pile_top_max_m": float((grain_xyz[mask, :, 2] + radii).max()),
            "lip_over_particle_column_samples": int(valid.sum()),
            "min_lip_minus_particle_surface_m": float(gap[valid].min())
            if valid.any()
            else None,
            "max_lip_minus_particle_surface_m": float(gap[valid].max())
            if valid.any()
            else None,
        }
    i = int(clearance.argmin())
    initial = states["initial_pile"]
    result = {
        "scope": "recorded-state geometry, not a rerun or contact-force validation",
        "bundle_sha256": manifest["sha256"],
        "ground_z_m": 0,
        "particle_count": len(grain_ids),
        "initial_pile_bounds_m": [
            list((initial - radii[:, None]).min(axis=0)),
            list((initial + radii[:, None]).max(axis=0)),
        ],
        "minimum_bucket_ground_clearance_m": float(clearance[i]),
        "minimum_clearance_time_s": float(states["time_s"][i]),
        "below_ground_samples": int((clearance < 0).sum()),
        "phases": phases,
        "limits": "21 points across lip width; actual sphere-column surface where present, not continuous soil; all box corners and compiled side-mesh vertices for ground clearance; floor z=0; observation samples may miss intervening extrema",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "spatial_report.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    np.savez_compressed(
        output / "spatial_trace.npz",
        time_s=states["time_s"],
        pile_top_m=(grain_xyz[:, :, 2] + radii).max(axis=1),
        bucket_ground_clearance_m=clearance,
        lip_minus_particle_surface_m=np.where(np.isfinite(gap), gap, np.nan),
    )
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--bundle",
        type=Path,
        default=Path("outputs/wa320/sensitivity_investigation/full_newton/bundle"),
    )
    p.add_argument("--states", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(audit(a.bundle, a.states, a.output), indent=2))
