# WA320 physics-backend benchmark

## Inspection before implementation

- Reference commit: `01df457`; existing MuJoCo M2 code is preserved. Entry points:
  `scripts/check_scripted_scoop.py`, `run_scripted_scoop_suite.py`,
  `evaluate_scoop_controller_comparison.py`, `render_scripted_scoop.py`.
- Domain controller: `control/scripted_scoop.py`; simulation coupling and force
  extraction: `environment/simulation/scoop_probe.py`; preparation: `terrain/bed.py`;
  material-ID/mouth accounting: `evaluation/material_accounting.py`.
- Vehicle: unchanged `WA320_reconstructed_v1` from `97ae4e5`, parameters
  `configs/vehicle/wa320.yaml`, committed original URDF/USD and manifest under
  `assets/wheel_loader/wa320/`. M2 consumes the passing Newton-exported complete
  floating MJCF, retaining all physical cylinders and seven connect constraints.
  The USD alone is not an equivalent numerical model: the existing Newton
  preparation supplies analytical tire cylinders, loop/limit references and contact settings.
- Material: independent free rigid spheres; radius 0.114–0.12 m, solid density
  2590 kg/m³ in `small_debug`. The preserved debug release has 83 elements,
  approximately 1435.297 kg. It is uncalibrated coarse material, not literal sand.
- M2 trial grain contacts: sliding friction 0.65; torsional/rolling length-valued
  friction 0.0096/0.0288 m; 4 ms contact reference, damping ratio 1,
  impedance 0.95/0.99, six-dimensional contact. Grain/bucket explicit pairs use
  sliding 0.3 and torsional/rolling 0.0096/0.0096 m. Vehicle/floor/constraints
  retain the exported model's values. No values will be independently fitted.
- Control: settle, approach, align, lower, penetrate, curl, lift, reverse, stabilize.
  Existing finite first-order actuator forces act on five physical sliders and
  four wheel motors. No independent bucket-angle drive. Legacy and stop/curl
  variants are diagnostic alternatives; all remain uncalibrated.
- Runtime: MuJoCo 3.12.0; 240 Hz control, eight physical substeps, implicit-fast,
  CG with 1000 iterations in M2. Existing state logs are 20 Hz; contact and mouth
  observations have distinct sampling schedules. Core accounting and geometry
  can be reused without importing Isaac/ROS into domain modules.
- Existing `newton_ground.py` uses Isaac Lab 3/Newton 1.6.1 and MJWarp, with
  force-driven v1 controls and export. It handles rigid ground/optional static
  pile, not transferable grains. The old `physx_probe.py` uses position drives
  and v0-specific assumptions; it must not be presented as a v1 scoop comparison.
- Installed reference environment has Isaac Lab 3.0.0, Newton 1.6.1,
  MuJoCo/MJWarp 3.12.0, Warp 1.17.0. A separate full Isaac Sim 6.2.0.0
  environment has Warp 1.19.0 and no installed Isaac Lab distribution. Keep
  these installations intact; inspect compatibility before launch, not a broad upgrade.

## Deterministic experiment

Use the existing `small_debug` scoop to bound scope. Capture its physical nine-
actuator effort commands at 240 Hz, initial complete state and compiled model.
Replay that immutable tape for exactly its recorded number of ticks in every
backend. Do not rerun a feedback controller independently per backend: that
would produce different actuator commands and confound the physics comparison.
The common release, including the ten-second settling interval, is identical;
backend-specific settling outcomes are results, not reset corrections.

Compare to native MuJoCo, not to a real WA320 measurement. Trajectory errors
therefore mean **sim-to-sim discrepancies**, never physical accuracy. Backend
startup/JIT, physics-loop time and instrumented end-to-end time must be separated.
Missing/unsupported fields are unavailable, not zero. Capacity, constraints,
initial-state and mapping failures invalidate a matched comparison.

## Mapping limits to audit

- PhysX Coulomb friction has no one-to-one mapping for MuJoCo `solref`, `solimp`
  or the full six-dimensional length-valued torsional/rolling law. Matching a
  sliding coefficient does not match the material model. Do not tune damping or
  friction independently to reproduce a payload.
- The analytical cylinder contact boundary in the M2 export may be cooked as a
  convex collider by PhysX. Validate and disclose the runtime boundary; identical
  radius/width metadata alone is insufficient.
- Isaac Lab scene reset, closed-loop pins, articulation joint ordering, inertia
  frames, torque/linear-force units and force sensor semantics need explicit checks.
- Direct MJWarp replay can isolate CPU/GPU execution of the *same* MJCF. It is
  not evidence that Newton's USD importer or a Newton MPM/DEM soil coupling works.

Primary references: [Newton 1.6 solver mappings](https://newton-physics.github.io/newton/1.6.0/solvers/mujoco.html),
[Isaac Lab 3 PhysX backend](https://docs.nvidia.com/isaaclab/3.0/concepts/isaac-sim-phys-x),
[PhysX mass and contact behavior](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/RigidBodyDynamics.html).
