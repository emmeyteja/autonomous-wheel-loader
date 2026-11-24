"""Read active contact forces directly from MuJoCo-Warp, bypassing host remapping."""

import numpy as np


class DeviceContacts:
    def __init__(self, model, gpu_model, gpu):
        import warp as wp

        self.model, self.gpu_model, self.gpu = model, gpu_model, gpu
        self.ids = wp.array(np.arange(gpu.naconmax), dtype=int, device=gpu.qpos.device)
        self.forces = wp.zeros(
            gpu.naconmax, dtype=wp.spatial_vector, device=gpu.qpos.device
        )
        self.rows = []

    def sample(self):
        import mujoco_warp as mw

        g = self.gpu
        count = int(g.nacon.numpy()[0])
        if count > g.naconmax:
            raise RuntimeError("GPU contact capacity exceeded")
        mw.contact_force(self.gpu_model, g, self.ids, True, self.forces)
        force = self.forces.numpy()[:count]
        geom = g.contact.geom.numpy()[:count]
        addresses = g.contact.efc_address.numpy()[:count, 0]
        world = g.contact.worldid.numpy()[:count]
        if not np.isfinite(force).all():
            raise RuntimeError("Nonfinite direct device contact forces")
        if np.any(abs(force[addresses < 0]) > 1e-8):
            raise RuntimeError("Inactive contact has nonzero device force")
        active = (addresses >= 0) & (world == 0)
        self.rows.append(
            [
                count,
                int(active.sum()),
                int((addresses < 0).sum()),
                float(abs(force).max(initial=0)),
            ]
        )
        return [
            (
                int(self.model.geom_bodyid[a]),
                int(self.model.geom_bodyid[b]),
                f[:3].astype(float),
            )
            for (a, b), f in zip(geom[active], force[active])
        ]
