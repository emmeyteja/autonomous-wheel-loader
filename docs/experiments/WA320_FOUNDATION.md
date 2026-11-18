# WA320 foundation validation

Original estimated primitive model; a physical foundation experiment, not a calibrated WA320 digital twin. Configuration: `configs/vehicle/wa320.yaml`. Portable assets: `assets/wheel_loader/wa320/usd/`. No Komodo inputs or dependencies.

## Passed

- Pure Python: 48 tests, including a 693-pose linkage sweep, independent pin reconstruction, singularity rejection, no-slip articulated wheel equations, parameter checks, archive isolation and PPO gating.
- Fresh core installation: same tests and `train.py --vehicle wa320` run without Isaac Lab, Isaac Sim, ROS, Torch or archived packages installed.
- Optional archive regressions: 70 tests pass after relocation.
- Two USD authoring tests: both portable assets have 10 bodies, 9 tree joints, nominal total mass 15,175 kg, mandatory passive closure, no bucket/rod drives, four explicit 128-vertex tire contact hulls, and no external/private paths.
- Xacro expands to the same 10-link, 9-joint tree. Closure remains a required sidecar/conversion step.
- Isaac Sim 6.2 / PhysX suspended fixture: wheels rotate, central steering articulates, boom lifts, bucket moves passively through the Z-bar. No reported self-contact along this test path. Maximum closure gap approximately 9 µm; passive-angle error approximately 0.000023 rad.
- Closure positive control: disturbing the passive rod creates about 0.317 m pin separation; the unactuated loop recovers to approximately 3 µm. This checks physical constraint enforcement independently of driven joint targets.
- Floating rigid-ground test: forward travel, straight-heading/lateral-drift gates, central articulation and sustained turning pass. Maximum closure gap approximately 4 µm.
- Rigid-pile approach: forward travel and measured bucket-to-pile contact pass; maximum closure gap approximately 4 µm. The pile is immovable and cannot supply bucket material.

Thresholds in the probes: closure gap ≤5 mm, passive-angle discrepancy ≤0.03 rad, nominal joint-limit excursion ≤0.005 rad, straight-heading error <0.05 rad, straight lateral drift <0.1 m, sustained turning >0.05 rad, ground penetration ≤10 mm, nonadjacent self-penetration ≤2 mm. These are diagnostic gates, not manufacturer tolerances or real-machine accuracy targets. Tests cover the recorded trajectories, not all combinations of limits or collision-free operating space.

## Newton fixture correction

- Previous baseline failed: 0.119 m maximum loop gap, 0.294 rad passive-angle error and boom motion 0.055 rad below its lower stop.
- Resolved numerical causes: coarse solver stepping and compliant default loop/stop constraints. Settings must be authored before scene construction; changes after import do not reach the solver.
- Profile: 240 Hz control, eight substeps (1,920 Hz solver), 2 ms loop/stop reference time and 0.9999 stop impedance. Geometry, masses and actuator limits are unchanged.
- Nominal maximum loop gap: 25.2 µm; passive-angle error: 0.0000614 rad. A 0.317 m deliberate pin displacement recovers to 21.3 µm without passive-link drives.
- All six driven-joint stop challenges pass the existing 0.005 rad excursion gate. Worst excursion: 0.00364 rad. Worst loop gap during these challenges: 0.554 mm.
- Two identical reset/replay trajectories match exactly. Resetting one of two environments leaves the other's joint positions, velocities and body poses unchanged.
- Sixteen-substep confirmation passes the same closure/stop/reset gates. Maximum nominal joint-trajectory difference from eight substeps: 0.000298 rad.
- Collision sweep: 5,760 recorded nominal/stress states have no nonadjacent penetration deeper than 2 mm. An artificial overlap is detected with collision filters unchanged. Minimum nominal bucket-lip clearance: 68 mm relative to the wheel-ground reference.
- Effective exported solver settings are audited, including unactuated passive links. Measurements are taken at control ticks; substep peaks and the full operating workspace are not covered.

This is numerical validation of an estimated model, not physical calibration. The numerical profile follows the [MuJoCo constraint model](https://mujoco.readthedocs.io/en/latest/modeling.html).

## Newton rigid-ground baseline (before correction)

Resolved diagnostic defect: [wheel-force continuity](https://github.com/emmeyteja/autonomous-wheel-loader/issues/5).

- Floating asset, locally authored analytic plane, unchanged estimated vehicle/contact parameters; same eight-substep numerical profile. No downloaded scene is required.
- Fifteen-second scenario: settle, ramp to 0.6 m/s rear-axle reference, drive straight, then ramp central articulation to 0.3 rad. The existing articulated wheel reference uses measured articulation and its rate.
- Forward displacement: 5.236 m. Straight-section maximum heading: 0.000510 rad; lateral drift: 1.12 mm. Sustained-turn yaw change: 0.184 rad. These pass the existing travel/steering gates; speed-tracking accuracy is not calibrated.
- Maximum linkage gap: 1.119 mm including settling. Joint limits, wheel rotation, 10 mm ground-penetration and 2 mm self-penetration gates pass at recorded control ticks.
- Previous baseline failure: all four measured upward wheel forces exceeded 1 N simultaneously at only 29.0% of control ticks after settling. The diagnostic requires at least 90%. Mean total upward contact force is approximately 148.94 kN versus nominal weight 148.87 kN; average load support does not establish continuous per-wheel support.
- Force samples come from the runtime contact sensor after each control tick. Collision distances are separately re-evaluated on CPU with the exported model and recorded floating-root/joint poses. Candidate contacts, geometric overlap and measured support force are reported separately.
- Box-floor tests, sixteen-substep refinement and multi-contact box-floor mode also failed baseline force continuity. These isolated changes did not establish a fix; see the combined correction below.
- Added CPU collision positive/negative controls: a clear floor reports no wheel contact; deliberate deep overlap fails penetration. Three integration tests pass in the Newton environment.
- Raw trajectories and logs stay local under ignored outputs. No measured tire, hydraulic or material calibration has been added.

## Wheel-contact correction

- An independent MuJoCo CPU run reproduced baseline force intermittency (approximately 28.8% four-wheel support), so the failure was not confined to the Isaac Lab sensor.
- Newton's initial joint state remained zero despite the configured transport pose. The ground probe now writes and verifies the intended closed pose before gravity/contact integration. The earlier run also included unintended linkage acceleration during settling.
- The derived Newton ground asset replaces the four 64-sided tire hulls with local-Y analytical cylinders of the same nominal radius and width. Original vehicle geometry/PhysX assets, body masses, COMs, inertias and drive limits remain unchanged. This removes facet-induced rolling-radius variation; no deformable tire or axle oscillation is added.
- Explicit ground configuration selects the Newton solver, elliptic friction cone and provisional 40 ms normalized contact reference with unit damping ratio. This is numerical contact compliance, not measured tire physics. Effective exported solver settings are audited.
- The combined correction passes every default ground gate: 98.78% simultaneous four-wheel upward force support after settling, 9.10 mm maximum penetration including the unchanged 20 mm spawn drop, 25.4 µm maximum loop gap, 5.986 m forward travel and 0.178 rad sustained-turn yaw change.
- Sixteen-substep confirmation passes the same travel/contact/closure gates: 98.81% force support, 9.00 mm maximum penetration and 25.7 µm maximum loop gap. This limited refinement check does not establish universal convergence.
- Floating reset writes joint/root poses and velocities, then repeats two stationary replays. Maximum reset joint error and replay component difference are both zero in the tested run; maximum closure gap is approximately 25.4 µm.
- Four integration tests pass, including preservation of source asset hashes, body mass properties, tire radius/width/axis and passive drives. The original suspended-fixture profile remains unchanged.
- `scripts/render_wa320_replay.py` renders saved floating poses to video without simulating new dynamics. Videos remain local; they show the approximation and do not prove physical accuracy.

## Incomplete

- Estimated rotary drives have load-dependent tracking error. No cylinder, hydraulic, driveline, tire-compliance or engine power-sharing validation exists.
- Full-workspace collision/stop coverage, randomized loaded operation and measured-trajectory calibration remain open. Newton rigid-ground diagnostic gates pass for the tested approximation; loaded/rough-terrain behavior and material interaction remain unvalidated. M1/M2 acceptance remains incomplete.
- The full algebraic joint envelope includes bucket poses below the ground. A task/controller must enforce workspace feasibility; independent joint bounds alone do not prevent ground contact.
- Granular terrain, scripted material scooping, PPO and world models are not implemented in the active system. Historical particle results do not satisfy these gates.

The solid initial front-frame proxy obstructed the linkage. A forked frame provides central clearance. Ground checks also improved after replacing backend-dependent cylinder collision tessellation with explicit tire hulls. Neither change calibrates real tire or excavation physics.

## Reproduce

- [Setup and probe commands](../SETUP.md).
- `python -m unittest discover -s tests/integration -v` in the standalone USD environment checks portable asset authoring.
- `scripts/check_wa320_physx.py` saves local motion/contact summaries and fails on unmet checks. `--capture` saves an optional verification image without policy camera observations.
- `scripts/check_wa320_isaaclab.py` checks the corrected Newton fixture and saves the derived asset, effective solver model, motion and validation summary. Use `--batch-reset` for reset isolation.
- Raw logs, poses, screenshots, videos and supplied papers remain local; `asset_manifest.json` records source/asset hashes for the committed model.
