# WA320 reconstructed v1 mechanical acceptance

- **Rigid-body mechanical checks pass. Expanded acceptance is incomplete:** penetration into deformable terrain is unsupported. `rigid_mechanics_passed=true`, `v1_complete=false`, `all_requested_validation_complete=false` in the local report. A rigid-pile contact pass is not excavation.
- This corrects the representation at commit `8c6ab13`: physical front/rear hinge housings and structural frames; explicit moving cylinders; direct linear cylinder forces; expanded static/chassis/dynamic tests. Earlier model results are not promoted to this revision.
- V0 remains unchanged in `configs/vehicle/archive/wa320_estimated_v0.yaml`, `configs/simulation/archive/wa320_estimated_v0_newton.yaml` and `assets/wheel_loader/wa320_estimated_v0/`.

## KNOWN / source distinctions

- Brochure references remain frozen: operating mass 15175 kg; wheelbase 3.030 m; track 2.050 m; wheel radius 0.695 m; body/tire width 2.590 m; clearance 0.425 m; articulation ±38.5°; selected bucket 3.0 m³ and 2.685 m width; length 7.720 m; cab height 3.200 m.
- Brochure workspace targets remain 4.005 m hinge, 5.590 m raised-rollback bucket height, 2.835 m dump clearance and 1.165 m reach. These are reference-pose comparisons, not an independently verified full workspace envelope.
- Service-manual component/cylinder dimensions and topology are verified against SEN06202-03. They are not measurements of a particular installed vehicle. Bucket-option and differently described assembly mass listings remain uncertain.
- No manufacturer CAD is used. [Mass budget, every body COM/inertia and source distinctions](WA320_MASS_DISTRIBUTION.md); [pin mapping, frame/cylinder geometry and equations](WA320_LINKAGE_RECONSTRUCTION.md); [parameter provenance](MEASUREMENT_PROVENANCE.md).

## DERIVED

- Closed component mass budget; compound primitive COM/inertia with parallel-axis and orientation transforms; axle moment balance; cylinder extension and neutral assembly coordinates.
- Empty transport total COM: **[−0.2597, 0, 1.0669] m** above ground. Predicted front: **61676 N / 6287.1 kg-equivalent / 41.43%**; rear: **87191 N / 8887.9 kg-equivalent / 58.57%**. These are model predictions, not manufacturer axle loads.
- The historical `boom_joint` / `tilt_joint` / `rod_joint` / `bucket_joint` graph preserves the manual four-bar relationships. Physical linear cylinders now transmit forces through their actual pinned/prismatic chains; bucket angle is passive.

## ESTIMATED

- Casting/rail/housing shape, pin clearances, unidentified attachment coordinates, transverse cylinder mounts, shell mass splits, all component COM placements/inertias, installed powertrain/equipment mass allocation and selected 3.0 m³ bucket mass/profile.
- Cylinder wall/shaft shape and 60/40 barrel/rod mass split; actuator tau, speed/force limits and bounded PI-D gains; tire/contact and low-speed wheel-drive approximation.
- Frame housings subdivide existing frame mass (160 kg rear, 110 kg front). Structural cuts/corridors and moved equipment recompute COM/inertia; mass is not silently removed or added.

## Performed checks and results

- **130 core tests:** component mass, full COM/inertia, hollow-ring inertia, static support/payload shift, frozen references, linkage geometry, normalized actuator lag/limits/reset, exclusive linear force application, neutral closure and offset rejection, independent URDF FK at all five articulation poses.
- **11 USD integration tests:** portable floating/fixed assets, unchanged source during numerical preparation, actual pin targets for the Z-bar and five cylinders, passive drives, numerical settings and replay frame resolution.
- **51-pose signature:** maximum algebraic Z-bar residual 6.7e−16 m; maximum analytical cylinder pin gap 5.0e−16 m. CSV includes boom/crank/bucket angles, tip position, absolute pin lengths and physical extensions. These are model-consistency residuals, not reconstruction accuracy.
- **19 native MuJoCo rigid-body cases:** acceleration, braking, steering, raise, curl, simultaneous drive/lift, 2000 kg payload carry, payload release and rigid-pile reaction; five static load poses; chassis at 0°, ±20°, ±38.5°. All pass their stated checks.
- Static cases check measured weight balance, individual positive wheel loads, equilibrium, COM-based front/rear reactions, actual slider strokes and pin gaps. Chassis cases check requested yaw, no relative hinge translation, wheel support, and self-contact.
- **Nominal Isaac Lab/Newton V-cycle:** all 13 phases pass, return error 0.342 m (0.35 m screen), maximum Z-bar gap 0.0091 mm. Independent FK audits all **53615** recorded states: maximum cylinder tip gap 0.0086 mm, maximum lift overrun 0.0046 mm (1 mm numerical screen); all five cylinders remain within stroke screens.
- Recorded hinge height 4.008 m; raised-rollback top 5.581 m; dump clearance/reach 2.863/1.207 m. Screening tolerances: 30/80/80/100 mm. Peak steering/lift/bucket forces: 27.8/184.7/289.2 kN per cylinder, below estimated 75/300/400 kN limits.
- Maximum transient bucket top remains 5.678 m. The approximate outline can exceed the brochure raised-rollback pose height between comparison poses; exact full-envelope fidelity remains unknown.
- Supplementary rigid-pile contact: peak bucket reaction ≈35.7 kN, maximum numerical overlap ≈0.82 mm. It is an immovable obstacle; these values do not establish digging resistance or material transfer.
- Traces record chassis velocity/pitch/yaw, articulation, individual wheel normals/slip, boom/bucket angles, cylinder lengths/forces, payload, actuator power/net work/positive work and mechanical energy. Contact losses, fluid losses, efficiency and shared power are not identified; actuator work is not a calibrated fuel/energy prediction.

## UNKNOWN / CALIBRATION_REQUIRED

- True pin-coordinate errors, the unidentified crank lower pin, exact installed equipment/bucket masses, COMs/inertias, rear-axle oscillation behavior, tire compliance/friction/slip, cylinder lag/force/friction/leakage and pump/engine power sharing.
- Bucket usable volume and attachment/material clearance. Passing an unloaded rehearsal or retaining a rigid ballast does not establish a clean fillable 3.0 m³ cavity.
- Deformable pile response, bucket penetration, acquisition/retention/spillage and material accounting. These cannot be validated with the current rigid terrain; the expanded request remains open for this test.
- Slow/wide cases retain controller regression coverage; this revision's Newton runtime evidence is the nominal case. No new claim of slow/wide runtime validation is made.

## Implementation and reproduction

- Modified: canonical vehicle/provenance config; vehicle generation, mass properties and FK; actuator force application; USD conversion/Newton preparation; existing dynamics/recording/replay integration; mass/linkage/status documentation and regression tests.
- Added: `evaluation/mechanical_signature.py`, `scripts/check_wa320_signature.py`, `scripts/render_wa320_hinge_checks.py`, `tests/test_wa320_mechanical_structure.py`, and original generated hinge-sector meshes. No unrelated refactor or new framework.

```bash
python scripts/build_wa320.py
python scripts/check_wa320_signature.py
python scripts/check_wa320_v_cycle.py --case nominal --output outputs/wa320/mechanical_correction/release_nominal
python scripts/check_wa320_dynamics.py --source outputs/wa320/mechanical_correction/release_nominal/effective_model.xml --output outputs/wa320/mechanical_correction/release_dynamics
python scripts/check_wa320_reconstruction.py --output outputs/wa320/mechanical_correction/mechanics.json
python -m pytest
```

- Conversion uses the existing isolated supported Isaac Sim importer; fixed/floating USD files are portable. Native diagnostics are independent CPU checks of the Newton-exported model, not a replacement simulator architecture.
- Numerical closure/contact settings and all approximations are explicit. No pose/velocity writes occur after physical initialization/reset. Offline FK/video replay writes recorded states only to inspect them; it does not advance or correct the physical run.
- Source PDFs, raw traces, plots, videos and local runtime logs stay outside Git. The public asset manifest records config/asset hashes.
- **Next milestone remains M2: deterministic scripted scoop with this reconstructed loader.** No PPO, world model, ROS/hardware, perception or material-model research was added here. Implement and test the missing material interaction before claiming scoop acceptance.
