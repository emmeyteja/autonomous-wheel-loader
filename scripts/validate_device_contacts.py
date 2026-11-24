"""Known-load active/inactive contact checks; independent of the WA320 controller."""

import json
from pathlib import Path

import mujoco
import mujoco_warp as mw
import numpy as np
import warp as wp

from wheel_loader.environment.simulation.device_contacts import DeviceContacts

if __name__ == "__main__":
    wp.init()
    results = {}
    for height, name in [(0.1, "resting"), (0.125, "inactive")]:
        model = mujoco.MjModel.from_xml_string(f"""<mujoco><option timestep="0.001" solver="Newton" iterations="1000" tolerance="1e-6"/>
        <worldbody><geom type="plane" size="1 1 .1" margin=".01" gap=".01"/>
        <body pos="0 0 {height}"><freejoint/><geom type="sphere" size=".1" mass="10" margin=".01" gap=".01"/></body></worldbody></mujoco>""")
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        gm = mw.put_model(model)
        gpu = mw.put_data(model, data, nconmax=64, njmax=128)
        reader = DeviceContacts(model, gm, gpu)
        for _ in range(1000 if name == "resting" else 1):
            mujoco.mj_step(model, data)
            mw.step(gm, gpu)
        mw.forward(gm, gpu)
        wp.synchronize()
        contacts = reader.sample()
        upward = sum(f[2] if b == 1 else -f[2] for a, b, f in contacts if 1 in (a, b))
        mujoco.mj_forward(model, data)
        native = 0.0
        for ci, c in enumerate(data.contact[: data.ncon]):
            if c.efc_address < 0:
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, ci, force)
            native += (c.frame.reshape(3, 3).T @ force[:3])[2]
        passed = (
            abs(upward - 98.1) < 0.5 and abs(upward - native) < 0.5
            if name == "resting"
            else upward == 0 and not contacts and reader.rows[-1][0] > 0
        )
        results[name] = {
            "device_upward_force_N": float(upward),
            "native_upward_force_N": float(native),
            "expected_force_N": 98.1 if name == "resting" else 0,
            "passed": bool(passed),
            "contact_audit": reader.rows[-1],
        }
    # A separated sphere inside contact margin tests inactive candidate handling.
    folder = Path("outputs/wa320/spatial_scoop/contact_validation")
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "report.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))
    if not all(r["passed"] for r in results.values()):
        raise SystemExit("Direct force validation failed")
