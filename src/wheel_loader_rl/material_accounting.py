"""Particle identity accounting for the prototype bucket; simulator independent."""
import numpy as np


def bucket_local_positions(points, pose):
    """World centres [N,3] -> bucket metres, pose [x,y,z,qw,qx,qy,qz]."""
    points, pose = np.asarray(points), np.asarray(pose)
    if points.ndim != 2 or points.shape[-1] != 3 or pose.shape != (7,) or not np.isfinite(points).all() or not np.isfinite(pose).all():
        raise ValueError('finite [N,3] centres and [7] pose required')
    q = pose[3:]
    if not np.isclose(np.linalg.norm(q), 1, atol=1e-3):
        raise ValueError('unit wxyz quaternion required')
    w, x, y, z = q / np.linalg.norm(q)
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return (points - pose[:3]) @ rotation


def bucket_contents(points, pose, radius_m):
    """Whole spheres in the open bucket interior; boundary spheres are excluded.

    Authored interior: x=[.04,1.4], y=[-.96,.96], z=[.02,.8] metres.
    Shape is tied to the documented approximation, not a real loader bucket.
    """
    if not np.isfinite(radius_m) or radius_m <= 0:
        raise ValueError('positive finite radius required')
    local = bucket_local_positions(points, pose)
    # Five millimetres allow the solver's small contact penetration.
    low = np.array([0.04, -0.96, 0.02]) + radius_m - 0.005
    high = np.array([1.4, 0.96, 0.8]) - radius_m + 0.005
    return np.all((local >= low) & (local <= high), axis=-1)


def transfer_metrics(positions, times, bucket_poses, radius_m, particle_mass_kg):
    """Count the same particle IDs at acquisition (16s), retention (22s), end.

    Destination is a demonstration ground pad x=[-2,1], y=[-2,2], top <= .6m.
    It is not a truck bed, autonomous dump zone, or a measured payload sensor.
    """
    p, t, poses = np.asarray(positions), np.asarray(times), np.asarray(bucket_poses)
    if p.ndim != 3 or p.shape[-1] != 3 or poses.shape != (len(p), 7) or t.shape != (len(p),) or len(p) < 2:
        raise ValueError('matching particle frames, times and bucket poses required')
    if not np.isfinite(particle_mass_kg) or particle_mass_kg <= 0 or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError('positive particle mass and increasing finite times required')
    masks = np.asarray([bucket_contents(frame, pose, radius_m) for frame, pose in zip(p, poses, strict=True)])
    result = {'measurement': 'whole-sphere containment with 0.005m contact tolerance; boundary grains excluded', 'complete_sequence': bool(t[-1] >= 30),
              'particle_count': p.shape[1], 'initial_mass_kg': p.shape[1] * particle_mass_kg}
    if t[-1] < 30:
        return result
    def at(seconds):
        index = int(np.argmin(abs(t - seconds)))
        if abs(t[index] - seconds) > 0.05:
            raise ValueError('recording must include acquisition and retention checkpoints within 0.05s')
        return masks[index]
    acquired = at(16)
    retained = acquired & at(22)
    released = retained & ~masks[-1]
    end = p[-1]
    pad = (end[:, 0] >= -2) & (end[:, 0] <= 1) & (abs(end[:, 1]) <= 2) & (end[:, 2] + radius_m <= 0.6)
    in_bucket = masks[-1]
    destination = ~in_bucket & pad
    source = ~in_bucket & ~pad & (end[:, 0] >= 1)
    other = ~(in_bucket | destination | source)
    final_counts = {'bucket': int(in_bucket.sum()), 'ground_pad': int(destination.sum()),
                    'source_region': int(source.sum()), 'other_or_spillage': int(other.sum())}
    for name, mask in (('acquired', acquired), ('retained', retained), ('released', released),
                       ('released_to_pad', released & pad), ('released_elsewhere', released & ~pad)):
        result[name + '_particles'] = int(mask.sum())
        result[name + '_mass_kg'] = float(mask.sum() * particle_mass_kg)
    result.update({'final_particle_counts': final_counts,
                   'conservation_error_kg': float((sum(final_counts.values()) - p.shape[1]) * particle_mass_kg),
                   'conservation_note': 'Identity partition of fixed particles; does not validate continuum soil mass or boundary payload.',
                   'transfer_screen_passed': bool(acquired.any() and retained.any() and (released & pad).any())})
    return result
