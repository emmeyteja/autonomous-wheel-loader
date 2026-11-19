# WA320 reconstructed v1 status

- V1 is an explicitly approximate rigid-body research model. The mechanical consistency gates pass for the nominal release run; manufacturer-CAD accuracy, real-machine dynamics and granular scooping are not established.
- Baseline: `configs/vehicle/archive/wa320_estimated_v0.yaml` , `configs/simulation/archive/wa320_estimated_v0_newton.yaml` and `assets/wheel_loader/wa320_estimated_v0/`, copied unchanged from commit `3fe8dab`. Active v1: `configs/vehicle/wa320.yaml` and generated `assets/wheel_loader/wa320/`. Baseline parameters/assets are never retuned to make v1 pass.
- Detailed reports: [mass budget, every body COM and inertia](WA320_MASS_DISTRIBUTION.md), [linkage, cylinders and actuator model](WA320_LINKAGE_RECONSTRUCTION.md), [source/error semantics](MEASUREMENT_PROVENANCE.md).

## KNOWN

- Frozen selected configuration: 15175 kg, 3.030 m wheelbase, 2.050 m track, 2.590 m tire/body envelope width, 0.425 m chassis clearance, ±38.5° articulation, 3.0 m³ / 2.685 m bucket, 7.720 m nominal length and 3.200 m cab height.
- Frozen workspace targets: hinge 4.005 m, bucket maximum 5.590 m, dump clearance 2.835 m and reach 1.165 m. Reach/clearance definitions and poses must match the brochure. Bucket width exceeds the chassis/tire width by design.
- Cylinder bore/rod/stroke/end-length references and selected component masses are verified against service-manual data. Manufacturer specifications are not our own real-machine measurements.

## DERIVED

- Mass accounting, component aggregation, primitive inertia transformations, full COMs, static axle equilibrium and virtual-work force transmission.
- Static empty transport result: total COM ≈[-0.251, 0, 1.059] m above ground; front 6332 kg-equivalent / 62.1 kN (41.72%), rear 8843 kg-equivalent / 86.8 kN (58.28%). These are sanity predictions, not manufacturer axle-load claims.

## ESTIMATED

- Unidentified pin/mount coordinates, bucket side profile and 3.0 m³ bucket mass proxy, installed rear equipment/powertrain masses, all body COM placements and inertias, cylinder barrel/rod mass splits, tire/contact response and low-speed drive behavior.
- First-order actuator lag, speed and force limits; force-transmission servo stiffness/damping and bounded integral hold gain; waypoint settings. Geometry, mass/inertia primitives, exterior collisions, actuators, contact and controller records remain independently configurable.
- Some legacy numeric fields remain for v0-compatible generation and diagnostics. V1 motion is force-driven with its own `actuators` section; legacy implicit position-drive gains do not actuate v1.

## UNKNOWN

- True errors of reconstructed geometry and mass properties; exact installed hardware/bucket-option match; detailed cylinder mass/COM movement; component COM/inertia measurements; sensor calibration and hydraulic pressure/flow time series.
- Bucket usable volume and attachment clearance for material loading. The unloaded V-cycle and a retained rigid ballast are not a granular scoop.

## CALIBRATION_REQUIRED

- Identify installed equipment masses/locations, validate axle reactions against weighing, and compare joint/cylinder transients with real telemetry.
- Identify actuator lag, speed/force envelopes, friction, tire slip, leakage and power sharing before claiming real-machine force accuracy. Do not infer these from a successful asset conversion or visually plausible motion.

## Performed checks

- Closed 15175 kg mass budget; explicit bodies and positive physical inertia tensors; independent parallel-axis/rotation tests; static moment balance and payload load-transfer checks.
- Multi-pose closure and fitted workspace checks; frozen reference regressions; manual cylinder dimensions; first-order analytic step, saturation, force caps, Jacobian finite differences and reset isolation.
- Earlier PD-only nominal Newton V-cycle passed the route checks but under-reached the hinge target by about 0.42 m. It is not current v1 completion evidence. The revised bounded integral-force release passes: route, curl/lift, reverse raised, dump, recurl/lower, return/stop, joint limits, physical loop closure, ground/wheel support, collision depth/self-collision and reset replay. Recorded actuator and measured workspace checks also pass.
- Seven native MuJoCo diagnostics of exported geometry: straight acceleration, braking, articulation, boom raise, bucket curl, 2000 kg rigid-ballast carrying and release onto a receiver. Total mass is conserved during release. These are separate from the Newton backend check.
- The CPU full-cycle diagnostic passes checks path, measured loop, cylinder limits and actual recorded workspace; a route-only pass is insufficient. No positions/velocities are overwritten after initialization/reset; only normalized-actuator-derived efforts are applied.

## Reproduction

```bash
PYTHONPATH=src python scripts/build_wa320.py
python -m pytest
PYTHONPATH=src python scripts/check_wa320_reconstruction.py
```

- USD conversion uses the existing isolated importer. Convert floating and fixed assets with `scripts/convert_wa320.py`; the active v1 USD has zero implicit drive gains because external first-order actuation supplies effort.
- In the Newton environment: `scripts/check_wa320_v_cycle.py --case nominal --usd <floating-v1.usda> --output outputs/wa320/reconstructed_v1/release_nominal`. Pass `--config configs/vehicle/archive/wa320_estimated_v0.yaml` and a baseline USD for a separate v0 comparison.
- `scripts/check_wa320_dynamics.py --source <exported-effective_model.xml> --output outputs/wa320/reconstructed_v1/release_dynamics` runs the seven CPU diagnostics. `scripts/check_wa320_force_cycle.py --source <exported-effective_model.xml>` runs the full CPU cycle. Render the recorded Newton motion with the existing replay script.
- The reconstruction report reads these locally generated cycle/dynamics artifacts; missing evidence, mismatched parameter fingerprints or failed checks keeps its completion flag false. Logs, telemetry, videos and supplied manuals remain outside Git.

## Stop boundary

- Stop after this v1 mechanical task. **Next: M2, deterministic scripted scoop using the reconstructed loader.** No scoop implementation, PPO, reward optimization, world model, ROS, perception or terrain research is added here.
- M2 must distinguish real material acquisition/retention from the current unloaded rehearsal and rigid-ballast test, and check the bucket/attachment cavity before relying on filled volume.

## Release measurements

- Nominal Newton cycle: 13 phases completed, return error 0.310 m, maximum loop gap 0.009 mm. No post-reset state overrides.
- Measured maximum hinge 4.008 m; raised-rollback top 5.581 m; dump clearance/reach 2.863/1.207 m. Screen tolerances are 30/80/80/100 mm respectively; these are acceptance screens, not estimated real-vehicle error bounds.
- Maximum transient bucket top is 5.678 m: the approximate outline can exceed the brochure's raised-rollback pose height during other bucket orientations. The report retains this separately; an independently verified full workspace envelope remains unknown.
- Peak steering/lift/bucket proxy forces: 26.9/184.9/291.6 kN per cylinder, below 75/300/400 kN caps. Largest nominal travel overrun is 0.004 mm at the soft lift stop, within the 1 mm numerical tolerance.
- Seven native CPU dynamics cases pass; 121 core tests and 11 USD integration tests pass. Archived v0 regenerates byte-identically (URDF, closure and meshes); its original USD files are retained unchanged.
- Evidence is the nominal reconstructed-loader case. Legacy slow/wide results are not promoted to v1 runtime evidence; those cases still have controller regression coverage.
