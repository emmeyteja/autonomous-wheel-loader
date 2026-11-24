"""Five-tick reproducer: audit inactive contacts without reading invalid host force rows."""

import argparse
import json
from pathlib import Path

import mujoco
import mujoco_warp as mw
import numpy as np
import warp as wp

from wheel_loader.environment.simulation.backend_benchmark import load_bundle


def check(bundle, output):
    model, tape, manifest = load_bundle(bundle)
    data = mujoco.MjData(model)
    data.qpos[:] = tape["initial_qpos"]
    data.qvel[:] = tape["initial_qvel"]
    mujoco.mj_forward(model, data)
    wp.init()
    gm = mw.put_model(model)
    gpu = mw.put_data(model, data, nconmax=4096, njmax=16384)
    rows = []
    for tick in range(5):
        wp.copy(
            gpu.ctrl,
            wp.array(tape["effort"][tick][None, :], dtype=wp.float32, device="cuda:0"),
        )
        for _ in range(manifest["substeps"]):
            mw.step(gm, gpu)
        mw.forward(gm, gpu)
        wp.synchronize()
        mw.get_data_into(data, model, gpu)
        n = int(gpu.nacon.numpy()[0])
        addresses = gpu.contact.efc_address.numpy()[:n, 0]
        ids = wp.array(np.arange(n), dtype=int, device="cuda:0")
        forces = wp.zeros(n, dtype=wp.spatial_vector, device="cuda:0")
        mw.contact_force(gm, gpu, ids, True, forces)
        values = forces.numpy()
        rows.append(
            {
                "time_s": (tick + 1) * manifest["control_dt_s"],
                "nefc": int(data.nefc),
                "contact_count": n,
                "device_inactive_contacts": int(np.count_nonzero(addresses < 0)),
                "host_contacts_with_address_beyond_nefc": int(
                    np.count_nonzero(data.contact.efc_address[: data.ncon] >= data.nefc)
                ),
                "direct_device_contact_forces_finite": bool(np.isfinite(values).all()),
                "direct_device_max_abs_force_torque": float(abs(values).max(initial=0)),
            }
        )
    output.mkdir(parents=True, exist_ok=True)
    result = {
        "scope": "short force-export reproducer, not a full-cycle force validation",
        "bundle_sha256": manifest["sha256"],
        "ticks": rows,
    }
    (output / "contact_export_check.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--bundle",
        type=Path,
        default=Path("outputs/wa320/sensitivity_investigation/full_newton/bundle"),
    )
    p.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/wa320/sensitivity_investigation/contact_export_check"),
    )
    a = p.parse_args()
    check(a.bundle, a.output)
