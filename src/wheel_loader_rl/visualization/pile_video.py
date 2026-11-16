"""Render recorded physical particle positions; never synthesizes motion."""
from pathlib import Path
import numpy as np


def render_pile_video(trace: Path, output: Path, fps: int = 25) -> Path:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    with np.load(trace) as data:
        positions = data['positions_m']
        times = data['time_s']
        radius = float(data['radius_m'])
        loader_poses = data['loader_body_poses'] if 'loader_body_poses' in data else None
        loader_names = data['loader_body_names'] if loader_poses is not None else None
    low, high = positions.min(axis=(0, 1)), positions.max(axis=(0, 1))
    grain_color_height = high[2]
    if loader_poses is not None:
        low = np.minimum(low, loader_poses[:, :, :3].min(axis=(0, 1)) - np.array([1.5, 1.2, 0]))
        high = np.maximum(high, loader_poses[:, :, :3].max(axis=(0, 1)) + np.array([1.5, 1.2, 1.5]))
    center = (low[:2] + high[:2]) / 2
    span = max(float((high[:2] - low[:2]).max()) + 0.6, 2.6)
    fig = plt.figure(figsize=(10.24, 6.4), facecolor='#101827')
    ax = fig.add_subplot(111, projection='3d', facecolor='#101827', computed_zorder=False)
    ax.set(xlim=(center[0]-span/2, center[0]+span/2), ylim=(center[1]-span/2, center[1]+span/2), zlim=(0, high[2]+0.25))
    ax.set_box_aspect((span, span, high[2]+0.25), zoom=1.25 if loader_poses is not None else 1.0)
    ax.view_init(elev=24, azim=-62)
    ax.set_axis_off()
    floor = [[(center[0]-span/2, center[1]-span/2, 0), (center[0]+span/2, center[1]-span/2, 0),
              (center[0]+span/2, center[1]+span/2, 0), (center[0]-span/2, center[1]+span/2, 0)]]
    ax.add_collection3d(Poly3DCollection(floor, facecolors='#29374b', edgecolors='#506078', linewidths=1.0, zorder=0))
    if loader_poses is not None:
        pad = [[(-2, -2, 0.005), (1, -2, 0.005), (1, 2, 0.005), (-2, 2, 0.005)]]
        ax.add_collection3d(Poly3DCollection(pad, facecolors='#23574f', edgecolors='#6ad6b7', linewidths=1.0, zorder=1))
        ax.text(-1.8, -1.8, 0.02, 'DUMP PAD', color='#a5ead5', fontsize=8)
    fig.text(0.06, 0.91, 'LOADER MATERIAL PROBE' if loader_poses is not None else 'PILE SETTLING', color='white', fontsize=22, weight='bold')
    fig.text(0.06, 0.86, 'Isaac Lab / Newton  ·  recorded physics positions', color='#9db0c7', fontsize=12)
    fig.text(0.06, 0.07, f'{positions.shape[1]} coarse spheres  ·  ' + ('scripted approximation; uncalibrated sand' if loader_poses is not None else 'gravity + contact; no loader yet'), color='#9db0c7', fontsize=11)
    timer = fig.text(0.78, 0.91, '', color='#e7b663', fontsize=16, weight='bold')
    u, v = np.meshgrid(np.linspace(0, 2*np.pi, 12), np.linspace(0, np.pi, 8))
    sphere = np.stack((radius*np.cos(u)*np.sin(v), radius*np.sin(u)*np.sin(v), radius*np.cos(v)), axis=-1)
    surfaces = []
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = FFMpegWriter(fps=fps, codec='libx264', extra_args=['-pix_fmt', 'yuv420p', '-crf', '21'])
    try:
        with writer.saving(fig, str(output), dpi=100):
            selected = np.unique(np.clip(np.searchsorted(times, np.arange(times[0], times[-1] + 0.5 / fps, 1 / fps)), 0, len(times)-1))
            for frame_index in selected:
                frame, seconds = positions[frame_index], times[frame_index]
                for surface in surfaces:
                    surface.remove()
                surfaces.clear()
                if len(frame) > 256:
                    # One point-cloud artist keeps dense previews practical. Every
                    # point still comes directly from the recorded physics state.
                    size = (2 * radius / span * (400 if loader_poses is not None else 600))**2
                    surfaces.append(ax.scatter(frame[:, 0], frame[:, 1], frame[:, 2],
                                               s=size, c=frame[:, 2], cmap='YlOrBr',
                                               vmin=0, vmax=grain_color_height, edgecolors='none',
                                               depthshade=True, zorder=2))
                else:
                    for index, point in enumerate(frame):
                        xyz = sphere + point
                        shade = 0.5 + 0.35 * ((index % 7) / 6)
                        surfaces.append(ax.plot_surface(xyz[..., 0], xyz[..., 1], xyz[..., 2], color=(shade, shade*0.66, shade*0.31), linewidth=0, antialiased=True, shade=True, zorder=2))
                if loader_poses is not None:
                    from wheel_loader_rl.visualization.loader_geometry import loader_faces
                    faces, colors = loader_faces(loader_poses[frame_index], loader_names)
                    surfaces.append(ax.add_collection3d(Poly3DCollection(faces, facecolors=colors, edgecolors='#303030', linewidths=0.25, zorder=3)))
                    from wheel_loader_rl.material_accounting import bucket_contents
                    mask = bucket_contents(frame, loader_poses[frame_index, list(loader_names).index('bucket')], radius)
                    if mask.any():
                        surfaces.append(ax.scatter(frame[mask, 0], frame[mask, 1], frame[mask, 2], s=50, color='#60e1ac', zorder=4))
                    from wheel_loader_rl.controllers.scripted_loading import loading_phase
                    timer.set_text(f'{seconds:4.1f} s\n{loading_phase(seconds)}')
                else:
                    timer.set_text(f'{seconds:4.2f} s')
                writer.grab_frame()
                if seconds == times[0]:
                    fig.savefig(output.with_suffix('.png'), dpi=100, facecolor=fig.get_facecolor())
    finally:
        plt.close(fig)
    return output
