# Loader approximation

- `loader_approximation.xml` is original project-authored primitive geometry, with no downloaded meshes or third-party machine asset.
- Simulation-only feasibility model; no manufacturer dimensions or validated machine dynamics.
- SI units; +X forward, +Y left, +Z up. Joints: `travel` [m], `lift` and `curl` [rad] about -Y.
- Chassis moves on a horizontal rail; wheels are fixed collision/visual geometry. No tire traction, wheel rotation, steering, suspension, or hydraulic model.
- Limits: travel [-1, 4] m; lift [0, 0.8] rad; curl [-1.3, 0.9] rad. Scripted target rates stay within 1 m/s, 0.4 rad/s, 0.6 rad/s.
- Masses: chassis/cab/wheels 6800 kg; lift arm 500 kg; bucket 470 kg. Approximate diagonal inertias [kg m²]: chassis [4000, 10000, 10000], arm [20, 250, 250], bucket [100, 150, 200], with body origins used as centres of mass.
- Bucket: open front/top, 1.4 m depth, 2 m width, 0.8 m height; 0.04 m side/back half-thickness and 0.02 m floor half-thickness. Grain containment excludes boundary grains, allowing 0.005 m contact penetration.
- Maximum drive force/torque: 100000 N / Nm. Position PD gains: travel 200000 N/m and 40000 Ns/m; attachments 2000000 Nm/rad and 60000 Nms/rad. These are feasibility assumptions.
- Loader contact friction 0.55; no calibrated sand–steel interaction. Self-collision disabled; ground and grain contacts enabled.
- A narrow USD authoring function builds this asset without an Isaac Sim importer. Generated USD and exported effective solver models stay under ignored outputs.
- Recorded body poses use `[x,y,z,qw,qx,qy,qz]`; the Isaac Lab 3 proxy's `xyzw` quaternion is converted at the simulator boundary.
