"""Common replay colors only; never changes physical model parameters."""

YELLOW = (0.95, 0.62, 0.05, 1.0)
STEEL = (0.60, 0.65, 0.67, 1.0)
BUCKET = (0.25, 0.27, 0.28, 1.0)
MATERIAL = (0.65, 0.49, 0.28, 1.0)


def apply_loader_palette(model):
    import mujoco

    for g in range(model.ngeom):
        body = int(model.geom_bodyid[g])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
        if not body or "wheel" in name:
            continue
        model.geom_rgba[g] = (
            MATERIAL
            if name.startswith("grain_")
            else STEEL
            if name.endswith("_rod") or "hinge_housing" in name
            else BUCKET
            if name.endswith("_bucket")
            else YELLOW
        )
