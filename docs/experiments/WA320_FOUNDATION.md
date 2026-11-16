# WA320 foundation validation

Original estimated primitive model; a physical foundation experiment, not a calibrated WA320 digital twin. Configuration: `configs/vehicle/wa320.yaml`. Portable assets: `assets/wheel_loader/wa320/usd/`. No Komodo inputs or dependencies.

## Passed

- Pure Python: 13 tests, including a 693-pose linkage sweep, independent pin reconstruction, singularity rejection, no-slip articulated wheel equations, parameter checks, archive isolation and PPO gating.
- Fresh core installation: same tests and `train.py --vehicle wa320` run without Isaac Lab, Isaac Sim, ROS, Torch or archived packages installed.
- Optional archive regressions: 70 tests pass after relocation.
- USD authoring check: both portable assets have 10 bodies, 9 tree joints, nominal total mass 15,175 kg, mandatory passive closure, no bucket/rod drives, four explicit 128-vertex tire contact hulls, and no external/private paths.
- Xacro expands to the same 10-link, 9-joint tree. Closure remains a required sidecar/conversion step.
- Isaac Sim 6.2 / PhysX suspended fixture: wheels rotate, central steering articulates, boom lifts, bucket moves passively through the Z-bar. No reported self-contact along this test path. Maximum closure gap approximately 9 µm; passive-angle error approximately 0.000023 rad.
- Closure positive control: disturbing the passive rod creates about 0.317 m pin separation; the unactuated loop recovers to approximately 3 µm. This checks physical constraint enforcement independently of driven joint targets.
- Floating rigid-ground test: forward travel, straight-heading/lateral-drift gates, central articulation and sustained turning pass. Maximum closure gap approximately 4 µm.
- Rigid-pile approach: forward travel and measured bucket-to-pile contact pass; maximum closure gap approximately 4 µm. The pile is immovable and cannot supply bucket material.

Thresholds in the probes: closure gap ≤5 mm, passive-angle discrepancy ≤0.03 rad, nominal joint-limit excursion ≤0.005 rad, straight-heading error <0.05 rad, straight lateral drift <0.1 m, sustained turning >0.05 rad, ground penetration ≤10 mm, nonadjacent self-penetration ≤2 mm. These are diagnostic gates, not manufacturer tolerances or real-machine accuracy targets. Tests cover the recorded trajectories, not all combinations of limits or collision-free operating space.

## Failing / incomplete

- Isaac Lab 3 / Newton / MuJoCo Warp loads the same fixed articulation and moves wheels, steering, boom and bucket, but fails the passive-closure, closure-recovery and joint-stop gates with the current settings. Measured maximum loop gap: approximately 0.119 m; passive-angle error: 0.294 rad; recovered disturbed gap: 0.173 m; minimum boom angle: −0.055 rad versus the configured zero stop.
- The exported Newton model contains two connect equalities for the loop; their presence does not establish sufficient constraint accuracy under load. Do not enable this backend for loading/training until its dynamics/constraint settings are resolved and tested.
- Estimated rotary drives have load-dependent tracking error. No cylinder, hydraulic, driveline, tire-compliance or engine power-sharing validation exists.
- Full mechanical-stop challenges, collision sweeps, repeatable reset/batch checks and measured-trajectory calibration remain open. M1/M2 acceptance is not complete merely because the PhysX diagnostic paths pass.
- The full algebraic joint envelope includes bucket poses below the ground. A task/controller must enforce workspace feasibility; independent joint bounds alone do not prevent ground contact.
- Granular terrain, scripted material scooping, PPO and world models are not implemented in the active system. Historical particle results do not satisfy these gates.

The solid initial front-frame proxy obstructed the linkage. A forked frame provides central clearance. Ground checks also improved after replacing backend-dependent cylinder collision tessellation with explicit tire hulls. Neither change calibrates real tire or excavation physics.

## Reproduce

- [Setup and probe commands](../SETUP.md).
- `python -m unittest discover -s tests/integration -v` in the standalone USD environment checks portable asset authoring.
- `scripts/check_wa320_physx.py` saves local motion/contact summaries and fails on unmet checks. `--capture` saves an optional verification image without policy camera observations.
- `scripts/check_wa320_isaaclab.py` reproduces the failing Newton fixture and saves the effective solver model.
- Raw logs, poses, screenshots, videos and supplied papers remain local; `asset_manifest.json` records source/asset hashes for the committed model.
