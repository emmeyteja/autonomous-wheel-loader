# Geometry-gated scoop diagnostic

## Implementation

- Opt-in `SpatialScoop` controller; original `ScriptedScoop`, M2 configuration and frozen effort benchmark remain unchanged. Reuse the original WA320 compiled scene, existing cylinder actuator proxy, wheel/steering interface, accounting and recorder. No new terrain framework, material tuning, particle scaling or training.
- Same controller configuration, gains, actuator limits, initial states and physics settings on native MuJoCo and direct MJWarp. MuJoCo Newton solver, 240 Hz control/observation, eight substeps. This is a feedback experiment: actual physical efforts differ and are recorded. It is separate from identical-effort replay.
- Live inputs: cutting-edge pose, lowest bucket collision point/velocity, physical cylinder states, settled particle XYZ/radii, active bucket/material reaction and geometrically measured in-bucket mass. These are privileged simulator measurements, not validated real sensors.
- Entry plane is selected from the settled particles intersecting the configured attack-height plane within the bucket's lateral span. Freeze that initial entry reference for penetration distance; measure contacts, pile top and occupancy continuously afterward. Current experiment assumes the preserved straight approach along world +X and centered debug pile, not arbitrary pile alignment.
- Sequence: settle/hold → approach with measured attack pose → penetrate → curl only after material contact and measured entry (or a measured stall after minimum entry) → lift only after curl progress and positive occupancy → reverse → retention check. Empty or stalled trials time out and fail.
- Curl holds lift-cylinder references while the cutting edge sweeps upward. Holding cutting-edge height fixed during curl would drive the boom toward its lower stop. Height/pitch velocity limits scale together to preserve the direction toward a feasible task-space target.
- Solve the current equality-constraint tangent for lift/curl motion and map task-space rates to physical cylinder velocities. Commands use the existing finite force servo and wheel-speed controller; no passive bucket-angle drive or body/joint state writes.
- Clearance prediction brakes travel and constrains cylinder-rate commands; joint-range projection also restricts commanded motion. These are command guards, not a mathematical guarantee for a compliant contact solver. Validate actual post-step clearance, joint overruns and pin gaps independently.
- Common attack-height target 0.085 m: 0.040 m desired floor clearance + 0.030 m plate half-thickness + 0.015 m tracking allowance. Curl entry is deeper than the first coarse-particle row. These are provisional control targets, not fitted soil properties or proven WA320 operating settings.
- Bucket occupancy sums the known masses of coarse particles whose centres satisfy the existing bucket envelope test. This is not load-cell weighing, exact occupied solid volume, or calibrated fill factor; partially intersecting elements and the coarse resolution limit its interpretation.
- Low-boom curl and carry targets stay within the preserved linkage workspace. Full real-machine rollback/workspace validation remains separate; do not expand model limits to force a successful animation.

## Direct device contact measurements

- New opt-in reader uses `mujoco_warp.contact_force` with actual device contact indices/world IDs. It bypasses the inactive-contact host-export defect, excludes inactive candidates and fails on non-finite forces or nonzero inactive forces.
- Every observation can archive candidate/active/inactive contact counts. Existing all-substep GPU overflow audit remains enabled. Native observations use active contact rows for matching semantics.
- Known-load validation: a 10 kg resting sphere produces approximately **98.100006 N** upward on GPU and **98.100002 N** natively (expected 98.1 N). A nonempty inactive-contact candidate returns zero force in both engines. This validates the tested sign/units/inactive handling, not granular accuracy.
- The legacy host-export path remains available to reproduce the preserved benchmark; new spatial feedback runs explicitly select the device reader.

## Acceptance and results

Quantitative results are generated under the ignored experiment root. Evaluate controller completion, measured floor clearance, actual joint overruns, connect-pin gaps, vehicle tilt, finite forces, solver/capacity flags, retained occupancy and mass-accounting residual separately. A numerically completed run is not a successful scoop.

The first geometric trials identified two problems: incompatible low-edge-height/curl targets, then shallow entry that displaced grains without acquisition. The current sequence holds the boom during curl and uses a lower, deeper attack. Superseded trial outputs remain local and are not accepted comparisons.

- Unloaded: three native runs are identical, finish in 18.6542 s and meet all diagnostic gates. Direct MJWarp finishes at the same time; maximum bucket discrepancy is 0.00000503 m. Native minimum floor clearance is 0.078744 m. No sampled force is non-finite; no solver/capacity flags are reported.
- Loaded native (three identical repetitions): **failed acquisition**, `phase_timeout_curl` at 47.4708 s. Actual minimum floor clearance 0.049384 m; no joint overrun, maximum pin gap 0.00000537 m and maximum tilt 0.00619°. Curl starts at 0.400165 m measured entry; final entry is 0.950352 m. Peak observed bucket/material reaction is 20.2212 kN.
- Twelve of 83 elements contact the bucket (205.627 kg), but zero mouth entries, zero acquired mass, zero retained mass and zero spillage are recorded. All 1435.296790 kg remain outside the acquisition classifier; 398.025938 kg move outside the source region. Maximum accounting residual is zero. This is particle displacement, not loading.
- The recorded edge reaches its approximately 0.20 rad curl target and rises above the bottom particle row. The lower/deeper attack avoids the old high pass and floor overlap in this trial, but does not establish a valid cut. Distinguish shallow engagement/curl timing, a blunt 0.06 m collision plate and coarse ~0.24 m particle diameter using a small edge-entry/contact-normal fixture before changing material properties or scaling particles. These are hypotheses, not validated causes.
- Native loaded instrumented wall time is 571.88–580.47 s for 47.4708 simulated seconds (real-time factor 0.0818–0.0830); this includes recording and diagnostics. These single-environment, CPU-feedback timings do not establish batched training throughput.
- All three loaded native runs have identical recorded generalized positions, scalar velocities, actuator forces and accounting. Each fails acquisition at the same timestamp. The unchanged-controller GPU run also fails with zero acquisition/retention at 47.4625 s. It contacts the same twelve particles; minimum clearance is 0.049296 m, peak bucket reaction 21.3559 kN, maximum tilt 0.019783°, zero joint overrun and zero accounting residual. Maximum bucket discrepancy from native on their common interval is 0.0073744 m; maximum body discrepancy including particles is 0.058977 m. Curl begins 0.008333 s earlier. All seven motion/numerical gates pass in both engines.
- GPU contact audit covers 11391 observations: up to 183 candidates, 182 active contacts and 48 inactive candidates (maxima need not coincide). All returned forces are finite, inactive forces are zero, and the every-substep overflow union is zero. No solver warning is reported. This is one loaded GPU run; GPU repeatability is not established.
- GPU instrumented wall time is 1444.87 s (1185.52 s stepping), real-time factor approximately 0.03285. The timings include synchronous CPU feedback/diagnostics and one environment; they do not select a training backend.
- **M2 remains incomplete**; no calibrated digging accuracy is claimed. Successful unloaded motion does not validate acquisition, loaded lift/reverse or spillage.

## Recommendation

- Use the new guards and direct force reader for controlled entry diagnostics. Keep the frozen effort experiment for numerical comparison; do not compare feedback trajectories as identical applied efforts.
- Do not accept the current controller as a loading baseline: loaded lift, reverse retention and spillage have not been exercised. Agreement between engines on an empty result is not physical validation.
- Next isolate a single-particle cutting-edge entry and a small row with recorded contact normals, mouth clearance and plate thickness; then compare delayed curl against this preserved failed trial. Keep material properties fixed until the entry mechanism is understood. Changes to the cutting-edge collider require a separate reviewed geometry variant, not silent changes to reconstructed v1.
- No PPO, world models, material calibration, larger particle batches or backend selection based on these timings.
- Verification: 172 core tests plus seven additional mechanics integration checks pass; the two known-load/inactive-contact fixtures pass. Code formatting/lint checks pass. Reviewed replay clearly shows unloaded completion followed by the failed loaded trial.

## Reproduction

Prerequisite: the preserved bundle described in [the backend benchmark](PHYSICS_BACKEND_BENCHMARK.md). Example using the same configuration in every run:

```bash
PYTHONPATH=src .venv/bin/python scripts/validate_device_contacts.py
PYTHONPATH=src .venv/bin/python scripts/run_spatial_scoop.py --unloaded --seconds 45 --output outputs/wa320/spatial_scoop/shared_v3/unloaded/native_1
# Repeat into native_2 and native_3, then run with --backend mjwarp into mjwarp_1.
PYTHONPATH=src .venv/bin/python scripts/run_spatial_scoop.py --seconds 90 --output outputs/wa320/spatial_scoop/shared_v3/loaded/native_1
# Repeat native trials and use the unchanged config for --backend mjwarp.
PYTHONPATH=src .venv/bin/python scripts/evaluate_spatial_scoop.py --root outputs/wa320/spatial_scoop/shared_v3
PYTHONPATH=src .venv/bin/python scripts/render_spatial_scoop.py --run outputs/wa320/spatial_scoop/shared_v3/unloaded/native_1 --run outputs/wa320/spatial_scoop/shared_v3/loaded/native_1 --output outputs/wa320/spatial_scoop/shared_v3/spatial_scoop_replay.mp4
```

- Use fresh output directories; preserve failed trials. Configuration is copied into every report/manifest. Zero effort arrays in the new feedback bundle are storage placeholders; the live controller generates all applied efforts.
- `states.npz`: measured motion, forces, material accounting and pile state. `controller.npz`: start-of-control-tick observations, goals, physical cylinder-rate commands and reactions. State records describe the end of each tick; the controller observation leads them by one 1/240 s interval.
- `validation.json` per run plus `comparison.json`: measured acceptance gates and common-interval repeat comparisons. `unloaded_metrics.png` / `loaded_metrics.png`: cutting-edge height, floor clearance, payload and reaction histories. Mechanical work is actuator force/torque times motion; it is not fuel or hydraulic efficiency.
- Replay reads recorded qpos only, with the shared yellow palette. It does not correct the trajectory or add payload. Three native repetitions and an independent unchanged-controller GPU run are the intended small suite; no large batch or RL run.
