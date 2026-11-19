# M2 deterministic scripted scoop

## Scope and inspection

- Vehicle baseline: `WA320_reconstructed_v1`, commit `97ae4e5`. No vehicle geometry, inertia, cylinder dimensions, collision masks or loop topology is changed by this experiment. V0/V1 baselines remain available.
- `terrain/rigid_pile.py` generates a fixed trapezoidal prism; `rigid_obstacle.py` authors it as a static convex collision mesh. Reuse its dimensions/conventions, but it cannot deform, transfer mass or validate digging.
- `terrain/column.py` and `terrain/bed.py` already generate deterministic coarse spherical material; `column_probe.py` and `blade_probe.py` advance free-body grains in native MuJoCo. Reuse the staggered-lattice preparation, six-dimensional frictional contacts, seed conventions and diagnostic runtime. The added `ScoopPile` stays inside the existing terrain module; the active terrain default/registry is unchanged.
- Production gravel/dry-sand files contain deliberately unknown calibration values. Existing `material_column.yaml` contains diagnostic trial values, not validated materials. These must not be silently promoted to calibrated soil.
- V1 bucket collision geometry has separate side-wall extrusions and segmented floor/back plates. Its mouth is open. The original profile, width, mass and constraints are preserved; manufacturer capacity and real attachment clearances remain uncertain. Coarse spherical grains cannot resolve fine gaps or a true sand boundary layer.
- `reconstruction_probe.py` already provides native CPU dynamics checks of the Newton-exported floating loader, physical linear cylinder efforts, contact telemetry and rigid-ballast diagnostics. Ballast attachment/release cannot acquire material and is not used here.
- `newton_ground.py` provides Isaac Lab/Newton MJWarp rigid-body simulation and XML export. The experiment consumes that complete exported model using the existing native MuJoCo diagnostic path. **This is CPU execution, not evidence of Isaac Lab/Newton GPU material coupling.** No second simulator architecture, ROS stack or training system is introduced.
- Existing V-cycle logs/replay/evaluation assume a single free vehicle root or no grains. Reusing them unchanged would misclassify grain contacts as vehicle collisions. M2 records all free-body states and separates wheel, bucket/material and vehicle self-contact.
- Missing at inspection: loader/material coupling, scoop sequence, material-ID accounting, acquisition/retention/spillage tests, repeated initial-state evidence and a material-aware replay.

## Material choice and parameters

- Approach: every coarse element is an independent dynamic sphere with its own fixed mass, inertia, free joint and frictional contact. Grain/grain, bucket/grain, tire/grain and ground/grain interactions occur in the same rigid-body solve. Pile deformation means grain rearrangement, not deformation of individual spheres.
- No particle is inserted into the bucket, removed, welded, attached or given a prescribed trajectory. Vehicle payload mass is never assigned. Loader motion comes solely from wheel torque and the existing physical cylinder-force controller.
- `configs/experiments/scripted_scoop.yaml` owns preparation, material, controller and acceptance settings. Grain radius 0.12 m is an ESTIMATED coarse numerical resolution, not gravel size or sand grain size. Radii vary from 95–100% of nominal with a fixed seed.
- Nominal solid density 2590 kg/m³ is the existing Aoshima/Servin reference used by the column diagnostic. Density variants are explicit trial values. Grain sliding friction 0.65 is an existing uncalibrated diagnostic trial. Length-valued torsional/rolling friction is scaled from the column radius 0.0125 m to 0.12 m: 0.001 × 9.6 = 0.0096 m and 0.003 × 9.6 = 0.0288 m. This preserves diagnostic coefficient/radius ratios, not real gravel calibration. These are not AGX dimensionless rolling coefficients.
- Bucket/steel contact separately reuses `blade_force.yaml`'s unmeasured gravel/blade sliding trial 0.3, with radius-scaled torsional/rolling coefficients 0.0096/0.0096 m. Explicit grain/bucket pairs preserve vehicle-only contacts. Copying grain/grain friction to the bucket is not an identified steel contact.
- Contact time 4 ms, damping ratio 1, impedance 0.95–0.99 and six-dimensional contact are numerical choices. They do not identify Young's modulus, cohesion, grain crushing or real digging resistance. Vehicle reference settings remain unchanged; grain priority selects the trial for ground/other material contacts; explicit pairs select bucket/steel contact.
- Deterministic trapezoidal release uses configurable height, width, crest length and slope. Gravity settles it without a retaining wall. Preparation slope is not a fixed repose-angle constraint; actual settled shape and motion must be inspected.
- The existing native runtime uses MuJoCo 3.12.0; v1 control tick 1/240 s, eight physical substeps, implicit-fast integration, CG solver with a 1000-iteration cap. CG is reused from the existing material diagnostics; the vehicle-only Newton diagnostic used its exported default solver. The numerical change is explicit and linkage/stability screens are rerun. No dependency upgrade or experimental MPM coupling is required.
- Newton supports free joints and sphere collisions, while its MuJoCo converter convex-hulls collision meshes. A hollow bucket must retain separate collision pieces. See [Newton 1.6 solver mappings](https://newton-physics.github.io/newton/1.6.0/solvers/mujoco.html). Particle-based media and volumetric deformable bodies are distinct capabilities; [Isaac Lab deformables](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/concepts/deformables.html) does not by itself establish this experiment's GPU coupling.

## Controller

- Sequence: settle → approach → align cutting-edge pitch → lower → penetrate → curl while advancing → lift/break out → reverse → stabilize.
- Attack pitch is used only to choose a reachable initial linkage configuration and corresponding cylinder lengths. There is no physical bucket-angle actuator. All attachment feedback is expressed as cylinder lengths/velocity commands through `FirstOrderCylinders` and `joint_efforts`.
- Configurable approach speed/distance, cutting-edge pitch and lip-height plan, penetration depth/speed, curl-start fraction, curl-cylinder velocity, lift-cylinder velocity, bucket reaction safety threshold and reverse distance/speed. Nominal settings: approach 0.25 m/s for 0.5 m; attack −5°, lip 0.045 m; penetration 1.7 m at 0.4 m/s; curl starts at 90% depth; curl/lift velocity caps 0.035/0.06 m/s; reaction threshold 20 kN; reverse 1.5 m at 0.25 m/s. An outer bounded PI chassis-speed loop commands the existing finite wheel motor proxy under pile resistance; it adds no body force or state correction. The threshold is an estimated early-breakout trigger, not the manufacturer breakout-force specification.
- Phase transitions use measured chassis displacement, cylinder lengths, contact force and explicit stabilization time. A stalled phase times out; no fake completion through elapsed travel time. Safety stops preserve recorded failures.

## Accounting and metrics

- Persistent ID and compiled mass for every grain. Initial total is independently checked against density × sphere volume; vehicle mass remains 15175 kg.
- Bucket payload uses grain centers inside the extruded bucket-profile polygon, with half-plate-thickness measurement clearance, below its measurement mouth plane. The mouth plane is a classifier, not a collision lid. Each included center contributes its whole element mass, even if a surface protrudes across the mouth; no fractional volume is inferred. A whole-radius erosion would incorrectly reject grains resting in finite-penetration wall contact. The classifier is a measurement convention, not evidence of a clean 3.0 m³ cavity. Final payload also must be clear of the floor, contact-connected to the bucket through grains, and moving below 0.05 m/s after stabilization.
- Remaining pile mass includes **all never-acquired material**, including grains displaced outside the original footprint. The settled source bounding box, expanded by maximum radius, only provides auxiliary `remaining_in_source_zone_mass_kg` and `displaced_unacquired_mass_kg`. Displacement is not bucket spillage. `spilled_mass_kg` / `bucket_exit_mass_kg` count previously acquired IDs currently outside the bucket. Reacquisition is allowed without double counting. Cumulative acquisition means unique IDs ever entering the bucket, not a sum of repeated crossings.
- Exclusive instantaneous partition: `initial = remaining + bucket + spilled + delivered + boundary_loss + residual`. Delivery is zero in this scoop-only experiment; no dump zone is claimed. Explicit loss category covers invalid/below-floor/out-of-domain positions; nonfinite simulation states fail immediately. Conservation tolerance is relative 1e−9; a small residual alone does not prove acquisition or accurate soil physics.
- Logged at 20 Hz: full poses, phase, material masses, cylinder force/extension, four wheel slip velocities, wheel traction/normal forces, bucket reaction, chassis pitch/roll/yaw and cumulative mechanical actuator work. Contact, force, slip and closure extrema are sampled screens; finite-state safety is checked each 240 Hz control tick, with eight physical substeps. Work is effort × generalized velocity, including wheel motors, integrated at control ticks. The reported positive-work channel integrates the positive part of total net actuator power; it is not the sum of individual positive actuator work and can hide simultaneous driving/braking losses. Neither channel is fuel or real hydraulic energy consumption.
- Solid-volume fill factor is `retained_mass / solid_density / nominal_3_m3`. Bulk-equivalent fill remains unknown until bulk density/packing is established; the two must not be confused.
- Repeat runs require identical prepared and settled states plus bounded final-position and payload differences. Geometry/material/controller variants are scenario sensitivity checks, not controlled controller comparisons. A controller comparison must reuse the same scenario and initial-state hash.

## Reproduce

```bash
PYTHONPATH=src .venv/bin/python scripts/check_scripted_scoop.py \
  --source outputs/wa320/mechanical_correction/release_nominal/effective_model.xml \
  --scenario nominal --output outputs/wa320/m2/accounted_suite/nominal
```

- Generate the source using the documented v1 Newton V-cycle first. Do not substitute a fixed-base or pre-correction model. Outputs remain ignored; public configuration and code provide reproduction.
- Run the same command for `low_shallow`, `tall_deep`, and a second nominal run in `nominal_repeat`; keep them under the same suite folder. Then run `scripts/evaluate_scripted_scoop.py outputs/wa320/m2/accounted_suite` and `scripts/render_scripted_scoop.py outputs/wa320/m2/accounted_suite/nominal`. The suite currently exits unsuccessfully for the documented physical acceptance failures. Install existing `material-audit`, `geometry-audit` and `dev` extras for CPU checks, plots and replay.
- Numerical results are reported below. A script exits unsuccessfully if any single-run physical acceptance fails. Scenario/repeat evidence is required separately; a successful single run does not set `m2_complete=true`.

## Acceptance status and quantitative results

- **M2 is not complete.** Nominal/tall cases demonstrate real acquisition and retention; shallow-pile acquisition fails, and the accepted runs demonstrate no nonzero bucket spillage. Neither displaced material nor an invalid earlier trial is used to satisfy the spillage gate.
- `m2_cpu_reference_passed=false`; `isaaclab_newton_gpu_material_execution_verified=false`; material calibration is false. Active terrain selection still defaults to the rigid pile. No PPO or world-model work is authorized by these results.
- Authoritative local evidence is `outputs/wa320/m2/accounted_suite/`; earlier development folders are not acceptance evidence.

### nominal

- Preparation height/width/slope: 0.70 m / 1.20 m / 32°; density 2590 kg/m³; 83 elements; attack -5°; depth target 1.70 m.
- Actual settled height/width: 0.613/1.846 m; grain speed RMS 0.000373 m/s. Discrete layer spacing means requested height is not attained exactly; nominal/tall preparations resolve almost the same height. Actual repose slopes are not identified at this resolution.
- Initial mass 1435.297 kg = remaining 1332.988 + bucket 102.309 + bucket spillage 0.000 + delivered 0.000 + boundary loss 0.000 kg, within floating-point residual. Displaced never-acquired material: 793.890 kg, included in remaining mass.
- Retained payload 102.309 kg; solid-volume fill factor 1.317%; cycle 33.308 s; forward/reverse travel 2.589/1.664 m. Travel overshoot is physical; the requested reverse distance is not imposed as a pose.
- Sampled peak wheel traction 20.356 kN; maximum slip velocity 0.0149 m/s. Steering/lift/bucket cylinder force peaks 20.771/82.186/34.128 kN; extension ranges and all five physical slider checks are in the report/recording.
- Net/positive mechanical actuator work 28.253/38.020 kJ; maximum chassis pitch/roll 0.00330°. This is not real fuel consumption or validated suspension behavior.
- Maximum cylinder/Z-bar pin gaps 0.0221/0.0717 mm against 1 mm screens; no slider overrun, bucket/floor overlap, vehicle self-penetration or solver warning. Single-run acceptance: **PASS**.

### low_shallow

- Preparation height/width/slope: 0.50 m / 1.00 m / 28°; density 2400 kg/m³; 36 elements; attack -6°; depth target 1.80 m.
- Actual settled height/width: 0.427/1.123 m; grain speed RMS 0.000179 m/s. Discrete layer spacing means requested height is not attained exactly; nominal/tall preparations resolve almost the same height. Actual repose slopes are not identified at this resolution.
- Initial mass 577.466 kg = remaining 577.466 + bucket 0.000 + bucket spillage 0.000 + delivered 0.000 + boundary loss 0.000 kg, within floating-point residual. Displaced never-acquired material: 561.156 kg, included in remaining mass.
- Retained payload 0.000 kg; solid-volume fill factor 0.000%; cycle 33.029 s; forward/reverse travel 2.637/1.727 m. Travel overshoot is physical; the requested reverse distance is not imposed as a pose.
- Sampled peak wheel traction 10.098 kN; maximum slip velocity 0.0200 m/s. Steering/lift/bucket cylinder force peaks 9.138/82.186/27.235 kN; extension ranges and all five physical slider checks are in the report/recording.
- Net/positive mechanical actuator work 22.371/31.719 kJ; maximum chassis pitch/roll 0.00183°. This is not real fuel consumption or validated suspension behavior.
- Maximum cylinder/Z-bar pin gaps 0.0331/0.0598 mm against 1 mm screens; no slider overrun, bucket/floor overlap, vehicle self-penetration or solver warning. Single-run acceptance: **FAIL — no material acquired/retained**.

### tall_deep

- Preparation height/width/slope: 0.80 m / 1.50 m / 36°; density 2700 kg/m³; 111 elements; attack -4°; depth target 1.90 m.
- Actual settled height/width: 0.618/2.077 m; grain speed RMS 0.000428 m/s. Discrete layer spacing means requested height is not attained exactly; nominal/tall preparations resolve almost the same height. Actual repose slopes are not identified at this resolution.
- Initial mass 2019.370 kg = remaining 1889.433 + bucket 129.937 + bucket spillage 0.000 + delivered 0.000 + boundary loss 0.000 kg, within floating-point residual. Displaced never-acquired material: 1417.203 kg, included in remaining mass.
- Retained payload 129.937 kg; solid-volume fill factor 1.604%; cycle 34.392 s; forward/reverse travel 2.920/1.658 m. Travel overshoot is physical; the requested reverse distance is not imposed as a pose.
- Sampled peak wheel traction 10.573 kN; maximum slip velocity 0.0149 m/s. Steering/lift/bucket cylinder force peaks 9.474/82.186/42.657 kN; extension ranges and all five physical slider checks are in the report/recording.
- Net/positive mechanical actuator work 34.801/44.703 kJ; maximum chassis pitch/roll 0.00261°. This is not real fuel consumption or validated suspension behavior.
- Maximum cylinder/Z-bar pin gaps 0.0309/0.0844 mm against 1 mm screens; no slider overrun, bucket/floor overlap, vehicle self-penetration or solver warning. Single-run acceptance: **PASS**.

### Repeatability and accounting

- Nominal versus `nominal_repeat`: identical prepared/settled hashes; maximum difference over **all recorded generalized positions** 0.0; payload difference 0 kg. Screens: 1e−7 for generalized positions and 1e−7 kg for payload. This is same-runtime determinism, not cross-device or CPU/GPU parity.
- Maximum absolute mass residual over all cases: 2.27e-13 kg. Relative tolerance 1e−9 of initial mass. The exclusive fixed-ID partition conserves mass by construction; independent compiled sphere-volume/density and unchanged vehicle-mass checks catch different errors. A near-zero residual does not validate the physical material law or centroid classification accuracy.
- Pile deformation, bucket/material contact, prescribed sequence, cylinder/Z-bar/stroke screens and stability pass in all cases. Low/shallow still pushes material without acquisition. Both successful cases start empty, finish with payload above the floor, and retain physically contact-supported material after reverse.
- Nonzero spillage acceptance **fails**: maximum measured bucket exit is 0 kg in this suite. The accounting unit test exercises acquire → spill → reacquire → delivery/loss, but this does not substitute for a valid physical spillage run.

## Artifacts and limitations

- Each case: `model.xml`, `report.json`, `accounting.json`, `recording.npz` (full positions, IDs, masses, force/stroke/work/contact/slip traces). Authoritative mass accounting uses full-precision records; PLY exports are visualization files.
- Nominal replay/plots: `scripted_scoop_replay.mp4`, `bucket_payload.png`, `payload_vs_time.png`, `metrics.png`, `pile_before_after.png`, `initial_pile.ply`, `final_ground_material.ply`, `final_bucket_payload.ply`. Green grains represent the same payload classifier used by accounting; replay asserts highlighted mass matches the recorded bucket mass at every frame. Initial/final scatter plots include retained material; PLY ground/payload exports separate it explicitly.
- Replay uses recorded states only. Existing original tire and off-road surface appearance is reused without changing physics. The floor is rigid/flat with an off-road texture; the pile is dynamic. Appearance does not prove real-world force accuracy.
- Early 331-element trials were too slow for rapid CPU checks. Unscaled length-valued rolling friction caused excessive spreading. A CG/200 trial exceeded the 1 mm pin-gap screen; it is excluded. The final radius-scaled trial, distinct steel contact and CG/1000 profile are explicit assumptions, not calibration against real loading data.
- Low shallow piles and earlier higher attack-angle trials produced no load. Contact traces indicate material pushed ahead/aside. The blunt 60 mm collision plate, minimum cutting clearance, contact law and coarse resolution are candidate contributors; this is an inference, not a uniquely identified cause. No vehicle geometry or state was changed to force a fill.
- Next work: isolate cutting-edge/contact-clearance behavior and shallow-pile entry; obtain a valid nonzero spillage case; then establish Isaac Lab/Newton GPU coupling/parity before making a production task claim. Preserve and rerun the identical-state baseline. Do not start PPO.
