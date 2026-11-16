"""CPU checks of collision filters exported by the Newton articulation scene."""
import numpy as np


def check_collision_filters(model_path):
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    arm = next(i for i in range(model.njnt) if model.joint(i).name.endswith('_arm_joint'))
    bucket = next(i for i in range(model.njnt) if model.joint(i).name.endswith('_bucket_joint'))
    bucket_body = int(model.jnt_bodyid[bucket])
    arm_body = int(model.jnt_bodyid[arm])
    chassis_body = int(model.body_parentid[arm_body])
    pairs, max_depth = set(), 0.
    for qa in np.linspace(-.1, .32, 5):
        for qb in np.linspace(-.5, .9, 5):
            data.qpos[model.jnt_qposadr[arm]], data.qpos[model.jnt_qposadr[bucket]] = qa, qb
            mujoco.mj_forward(model, data)
            for contact in data.contact:
                a, b = (int(model.geom_bodyid[g]) for g in contact.geom)
                if a and b:
                    pairs.add(tuple(sorted((a, b))))
                    max_depth = max(max_depth, -float(contact.dist))
    # Positive control in a CPU-only copy: place one chassis collider inside the
    # bucket. This intentionally artificial overlap is never used in the GPU demo.
    chassis_geom = next(i for i in range(model.ngeom) if model.geom_bodyid[i] == chassis_body)
    bucket_geom = next(i for i in range(model.ngeom) if model.geom_bodyid[i] == bucket_body)
    original = model.geom_pos[chassis_geom].copy()
    world_position = data.geom_xpos[bucket_geom].copy()
    root_rotation = data.xmat[chassis_body].reshape(3, 3)
    model.geom_pos[chassis_geom] = (world_position-data.xpos[chassis_body]) @ root_rotation
    mujoco.mj_forward(model, data)
    detected = any({int(model.geom_bodyid[c.geom[0]]), int(model.geom_bodyid[c.geom[1]])}
                   == {chassis_body, bucket_body} for c in data.contact)
    model.geom_pos[chassis_geom] = original
    return {'runtime_export': 'Newton MuJoCo scene; checked with CPU MuJoCo collision detection',
            'sampled_joint_configurations': 25, 'self_contact_body_pairs': [list(pair) for pair in sorted(pairs)],
            'maximum_sampled_self_penetration_m': max_depth,
            'artificial_nonadjacent_overlap_detected': bool(detected),
            'original_geometry_validated': False}
