# WA320 mechanism sensitivity investigation

## Scope

- Preserve WA320 geometry, inertias, initial configurations, material parameters, M2 controller and original benchmark. No training or material calibration.
- Compare **MuJoCo's `mjSOL_NEWTON` solver** on native CPU and direct MuJoCo-Warp GPU. This is not NVIDIA Newton Physics, its USD importer, or its material coupling.
- Results are numerical diagnostics, not real-machine accuracy validation or evidence of physical instability.

## Initial constraint consistency

- The model has seven `connect` equalities (21 Cartesian residual components): two lift-rod/boom pins, bucket-rod/bellcrank pin, two steering-rod/front-frame pins, and two connecting-rod/bucket pins. Cylinders are physical slider joints; the bucket angle remains passive.
- The preserved release has maximum equality-position component **2.99617e-7 m**. Configurations are left unchanged, so the investigation does not claim exact positional closure.
- Zero initial velocity gives zero equality-velocity residual.
- Changing only bucket-slider velocity to **1e-7 m/s** gives maximum `|J_eq v|` **9.88720e-8 m/s**. It is not a valid closed-linkage tangent velocity.
- Construct a consistent perturbation by solving `J_eq v = 0` while prescribing bucket-slider speed 1e-7 m/s, holding the other four cylinder rates, root, wheels and grains at zero. Solve the remaining mechanism velocities by minimum-norm least squares in their SI joint coordinates. This is not a mass-weighted projection.
- Maximum consistent velocity residual: **2.57419e-21 m/s**. Bucket-joint, bellcrank/tilt, connecting-rod and cylinder-hinge rates move together; the bucket-joint rate is approximately **2.70457e-7 rad/s**. Boom velocity remains effectively zero.
- This is a tangent to the preserved, slightly imperfect release. No position projection, particle movement or extra generalized force is applied.
- Full initial vectors for baseline, isolated and consistent perturbations, Jacobian singular values and generalized constraint forces are written to each bundle's `benchmark.json`. Generalized force components have N for translation/sliders and N m for rotation/hinges; do not treat their combined norm as a force in newtons.

Initial native CG generalized constraint-force maxima, separating units:

- Fixed fixture: baseline slider **121.15348 N**, hinge **1687.48098 N m**; consistent slider **121.15352 N**, hinge **1687.48095 N m**.
- Floating ground: baseline slider **6.49138 N**, hinge **7.43142 N m**; isolated slider **6.49452 N**, hinge **7.43719 N m**; consistent slider **6.49138 N**, hinge **7.43142 N m**.
- Full-pile initial values match the floating unloaded values to rounding. Initial motor efforts are zero. These are initial solver reactions, not excavating forces.

## Ten-second isolation experiments

- All paired runs use identical effort tapes, initial configurations, timestep (1/1920 s), eight substeps, 240 Hz control/observation, solver settings and physical parameters.
- Fixed fixture removes grains, mounts the rear chassis and disables contacts. Ground fixture removes grains but retains floor/tire contact. Full scene retains the original 83 coarse spheres.
- Run **three zero-velocity native CG baselines per fixture**. Both repeats match the first baseline exactly in recorded body positions.
- Run isolated and consistent perturbations once per CG fixture. Additional paired consistent-perturbation Newton runs use the same inputs except the solver setting.
- Separately labelled actuator-off ablations set the entire effort tape to zero; they are not substitutes for the original identical-effort experiment.

Maximum bucket-origin difference from the corresponding zero-velocity baseline:

| Fixture | CG isolated | CG consistent | Newton consistent | CG consistent, zero efforts |
|---|---:|---:|---:|---:|
| Fixed, no contacts/grains | 0.00472 mm | 0.07027 mm | 0.07446 mm | 0.05888 mm |
| Floating, ground only | 228.919 mm | 202.773 mm | 3.93743 mm | 0.54341 mm |
| Full 83-sphere scene | 523.474 mm | 524.026 mm | 3.93744 mm | Not run |

- Additional contact ablation retains the **floating root** while disabling contacts and removing grains: CG consistent-perturbation discrepancy is **0.07323 mm**. The loader falls freely in this diagnostic; it is not a vehicle-operating scenario. This separates contact removal from the fixed fixture's chassis mount.

Interpretation:

- Invalid initial velocity is a real flaw in the old experiment, but correcting it does **not** remove the large CG full-scene divergence.
- Large divergence already exists without grains when tire/ground contact is present and becomes small when contacts alone are disabled on the floating vehicle. Granular contacts are not necessary for it. Collision-geometry reconstruction at all saved full-scene baseline/consistent poses found **zero grain-to-vehicle contact frames** during these 10 s; bucket/grain reaction is also zero. Adding grains changes the numerical system before physical digging, not an observed bucket/grain force response. This check cannot exclude contacts between observation instants.
- Both the Newton solver and removal of actuator efforts sharply reduce ground-case sensitivity. Evidence supports an interaction between contact solving and forced motion, not an identified physical instability.
- Replay motors have `na = 0`, with all nine actuator dynamic types `none`: no hidden first-order actuator state exists in this numerical replay. The external M2 actuator dynamics generated the tape once; they are not advanced differently per backend.
- The 12 principal equality-Jacobian singular values do not collapse in the CG isolation fixtures (12th stays above about 0.78 in the chosen mixed-SI coordinates). Additional tiny singular values and threshold-dependent rank reflect weak/redundant constraints and soft-constraint drift. This does not prove the linkage is free of singularities; it provides no evidence of a principal-rank collapse causing the observed divergence.
- Baseline maximum equality-position components are 0.0147/0.0543/0.0464 mm for fixed/ground/full scenes; maximum equality-velocity components are 0.00922/0.0332/0.0290 m/s. Small position gaps alone therefore do not establish dynamically identical motion.
- These controlled ablations identify contributing conditions; they do not isolate a particular tire contact, friction transition, or solver branch as the unique cause.

## Full-duration Newton CPU/GPU comparison

All **four full-duration runs completed**: two native and two direct MJWarp. Completed status means finite generalized positions/velocities and all requested ticks executed, not physical acceptance.

- Native repeat: maximum recorded body-position and bucket differences **zero**.
- MJWarp repeat: maximum bucket difference **1.37599 m**, RMS **0.50312 m**; maximum body-position difference **1.45094 m**. Full-cycle repeatability fails.
- Native versus GPU run 1: bucket maximum/RMS **0.72415 / 0.30019 m**; boom **0.70947 / 0.28569 m**. GPU run 2 versus native: bucket **0.67605 / 0.23096 m**.
- Run-1 rear-frame maximum position discrepancy **0.70949 m**; bucket expressed in its own rear frame differs by up to **0.35296 m**. Both travel and mechanism motion contribute.
- Run-1 maximum cylinder-position discrepancies: curl **0.12057 m**, lift left/right **0.07333 m**, steering left/right **0.01527 m**. Articulation discrepancy **0.04229 rad**.
- Native maximum connect-pin gap **0.01926 mm**; GPU run 1 **0.00343 mm**. Their small size does not establish trajectory parity. Maximum equality-velocity components are **0.02184 m/s** native and **0.000650 m/s** GPU run 1.
- All-step GPU overflow unions: **0 in both runs**. Native warning counters are zero for every warning type in both runs. No runtime solver/capacity warning lines were found in either GPU log. Sampled solver iterations peak at 8 native and 9 GPU run 1, below the cap.
- All runs end with **0 kg geometric bucket occupancy**. These outcomes do not validate material acquisition or digging accuracy.

- Same compiled-model, inspection-XML and command-tape hashes; 7994 ticks, 33.308333 s, 240 Hz observations, eight substeps, Newton/implicit-fast, iteration cap 1000, tolerance 1e-6. No separate parameter fitting.
- Identical **requested** initial state and efforts do not mean bitwise runtime equality: native uses double precision and installed MJWarp uses float32. Source-to-float32 maximum coordinate rounding is **2.34232e-7** in mixed position/quaternion coordinates; initial velocity rounding is zero. Maximum effort rounding is **0.003756** in mixed force/torque coordinates. These are representation differences, not tuned physical parameters.
- Initial constraint diagnostics in the bundle and `mechanism.npz` row zero describe the requested native release. They are not a measurement of the GPU's post-conversion initial reactions.
- Instrumented wall times are retained locally. Concurrent diagnostic/render jobs and instrumentation make them unsuitable for a throughput ranking.

### Contact-force export defect

- Both GPU runs contain six non-finite wheel-reaction components across observations at **0.01667 and 0.02083 s**. Raw traces are preserved; invalid values are not replaced by zeros.
- A separate five-tick reproducer identifies an installed MuJoCo-Warp host-export defect: `get_data_into` assigns sequential host force addresses even to inactive contacts. At tick 4 there are 14 inactive contacts and only the 21 equality rows, yet host addresses reach 78; tick 5 has 16 inactive contacts and addresses reach 90. Calling native `mj_contactForce` on those exported addresses is invalid.
- Audited future replays guard invalid exported force addresses and record unavailable values instead of reading beyond the force array. This does not repair the underlying mapping or change dynamics.
- Direct `mujoco_warp.contact_force` extraction returns finite zeros for those inactive contacts in the reproducer. This establishes the short export fault, not a validated replacement full-cycle force trace.
- **Withhold GPU contact-force metrics, including finite-looking peaks**, until direct device extraction and contact-index mapping are validated across a fresh full run. The evaluator marks these unavailable and preserves invalid-sample counts. Native contact forces remain separate.
- This observation defect does not feed forces back into either replay. Generalized GPU state and generalized constraint forces are copied independently; it cannot by itself explain the observed trajectory repeat failure. It also means zero overflow flags alone are insufficient to validate observation data.


## Separate bounded feedback diagnostic

- Ten-second floating/no-grain Newton fixture, same initial state, 240 Hz, identical reference trajectory from the native zero-velocity Newton run.
- Reference is the state at the beginning of each control tick. Feedback uses each motor's scalar joint position/velocity; wheel position gains are zero. Passive bucket angle is never commanded.
- Law: original effort tape feedforward plus `Kp*(q_ref-q) + Kd*(v_ref-v)`, clipped to common existing cylinder force and wheel torque caps. Cylinder stiffness/damping and wheel damping come from the unchanged vehicle configuration. No gains are fitted per backend.
- Record actual commands and resulting actuator forces for both engines. Reference-file hash and vehicle-parameter hash identify shared inputs; reproduction also records explicit gains and limits.
- Observed maximum native/MJWarp bucket discrepancy: **0.10687 mm**, RMS **0.03137 mm**. Maximum curl effort difference is **2.647 kN**: feedback produces different physical commands, so these results are separate from identical-effort replay.
- This is a diagnostic trajectory tracker with estimated gains, not an accepted scoop controller or hydraulic model. It establishes neither real-machine tracking nor loaded stability.

## Recorded evidence and reproduction

```bash
# Requires the preserved parent bundle from PHYSICS_BACKEND_BENCHMARK.md.
PYTHONPATH=src .venv/bin/python scripts/run_mechanism_investigation_suite.py --phase sensitivity
PYTHONPATH=src .venv/bin/python scripts/run_mechanism_investigation_suite.py --phase full-newton
PYTHONPATH=src .venv/bin/python scripts/run_mechanism_investigation_suite.py --phase feedback
PYTHONPATH=src .venv/bin/python scripts/check_mjwarp_contact_export.py
PYTHONPATH=src .venv/bin/python scripts/evaluate_mechanism_sensitivity.py
PYTHONPATH=src .venv/bin/python scripts/render_mechanism_investigation.py
```

- Default ignored output root: `outputs/wa320/sensitivity_investigation`. Use a fresh `--root` to repeat the whole matrix. Existing completed, matching runs are reused; input mismatches fail.
- Individual preparation/run entry point: `scripts/investigate_mechanism_sensitivity.py`. Supports baseline/isolated/consistent perturbations, fixture, solver, duration and explicitly labelled zero-effort ablation.
- `states.npz`: bucket/boom trajectories, all scalar joint positions and velocities (including all five cylinders and articulation), actual actuator forces, generalized constraint forces, wheel/ground and bucket/grain reactions, contact counts, material accounting and final pile positions.
- `mechanism.npz`: initial plus 240 Hz equality position/velocity/force residuals, generalized constraint forces, effort commands, Jacobian singular values, potential/kinetic mechanical energy and warnings. Built-in potential/kinetic energy excludes unmodelled hydraulic storage and is not fuel consumption.
- GPU audit unions overflow flags **after every physics substep and every observation forward pass**, independently of whether a later flag clears. It archives cumulative flags at every control tick. Native warning counters are checked every substep. Suite runtime logs and matching warning lines stay local.
- Plots: `sensitivity.png`, `full_newton.png`, `full_newton_cylinders.png`, `feedback.png`; machine-readable comparisons in `comparison.json`. Video replays recorded qpos without physics steps or state corrections.
- Software checks: **166 core tests + 7 integration tests passed**. Checks cover tangent construction, retained transient overflow flags, floating contact-ablation preparation and shared feedback limits. These are software contracts, not mechanical validation.

## Recommendation

- Do not resume quantitative material calibration, RL or payload-based backend selection yet. Preserve native as the diagnostic reference and the original identical-effort tape as a numerical stress test.
- The current CG contact/effort setup is too sensitive for attributing retained-payload differences to digging fidelity.
- **The present mechanism/numerical setup is not reliable enough to resume quantitative M2 digging.** Correcting the initial velocity is necessary, but insufficient: Newton full-cycle GPU repeatability and CPU/GPU parity still fail, and GPU contact-force export is invalid. This is not a proof of physical mechanism instability.
- Next fix and validate direct device contact-force logging, then extend the shared bounded feedback diagnostic to the full unloaded travel/lift cycle with repeatable inputs. Review weak/redundant linkage constraints and matched solver/contact sensitivity without changing geometry or fitting material to compensate. Retain open-loop stress tests alongside feedback tests.
- The shared feedback benchmark is a separate candidate for controlled motion comparisons. Extend it to loaded interaction only after repeatability, linkage constraints, contact behavior and bounded actuator motion pass explicit gates.
