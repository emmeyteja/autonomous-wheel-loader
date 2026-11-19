# Original WA320-like asset

- Source of truth: `configs/vehicle/wa320.yaml`; regenerate with `python scripts/build_wa320.py`.
- `wa320.urdf`: articulated tree with original parametric geometry and explicit mass allocation and estimated compound box/cylinder/ring inertias.
- `wa320.xacro`: wrapper expanding the generated tree. Parameters are edited in YAML, not duplicated in Xacro.
- `closure.json`: mandatory passive rod-to-bucket and five cylinder-tip closures, added by the supported conversion integration.
- `usd/wa320.usda` and `usd/wa320_fixed.usda`: portable PhysX assets with the passive closure and explicit tire contact hulls; floating driving model and suspended fixture respectively.
- No external CAD or mesh assets. No Komodo geometry. This is not a manufacturer model.
- Bucket capacity is reference metadata; compound-box shell geometry is not verified to hold 3 m³.
- USD conversion and validation instructions: [setup](../../../docs/SETUP.md).
- Tires use original generated meshes: rounded sidewalls, chevron lugs, yellow rims, hubs and bolts. Regenerate OBJ visuals with the same build command; no external assets are required.
- Tire detail is visual only. Cylinder/convex contact geometry, rigid wheel inertia and control limits are unchanged. This does not model tread traction, tire deformation or calibrated rubber.
- Replay uses the same procedural appearance over recorded motion. `render_wa320_replay.py` does not rerun dynamics.

- Hinge housings contain upper/lower pin supports and preserved bores. Each cylinder has a physical pinned barrel, prismatic rod and constrained tip; replay shows those physical bodies.
- [Mechanical acceptance and remaining unsupported terrain test](../../../docs/calibration/WA320_RECONSTRUCTION_STATUS.md). V0 remains under `../wa320_estimated_v0/`.
