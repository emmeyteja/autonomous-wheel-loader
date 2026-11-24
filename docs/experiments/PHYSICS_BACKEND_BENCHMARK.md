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

## Measured result and decision

**Sim-to-sim only. No backend has been validated against real WA320 digging.**
Follow-up [mechanics diagnostics](MECHANICS_BACKEND_DIAGNOSTICS.md) demonstrate
strong open-loop sensitivity before attack. The retained-payload values below
cannot rank digging fidelity; short unloaded solver parity is assessed separately.
The preserved 83-grain debug scoop ran for 33.3083 s with the same 7994 physical
nine-channel effort commands. Each backend was repeated once. Native replay
reproduced the captured controller run's recorded qpos exactly. No independent
payload tuning, training, particle scaling or replacement terrain framework was used.

| Metric | Native MuJoCo | Isaac Lab + PhysX CPU* | Direct MJWarp† |
|---|---:|---:|---:|
| Bucket origin trajectory RMS discrepancy [m] | 0 (reference) | 1.558 | 1.387 |
| Boom origin trajectory RMS discrepancy [m] | 0 (reference) | 1.517 | 1.250 |
| Peak sampled bucket/grain reaction [kN] | 12.385 | 12.715 | 5.336 |
| Peak individual wheel/ground reaction [kN] | 64.988 | 376.068 | 65.270 |
| Maximum initial-envelope penetration proxy [m] | 1.085 | 0 | 0 |
| Material contacted [kg, unique IDs] | 274.006 | 100.983 | 51.931 |
| Material entering/acquired [kg] | 102.309 | 0 | 0 |
| Final retained material [kg] | 102.309 | 0 | 0 |
| Extraction spillage [kg] | 0 | 0 | 0 |
| Pile-centre RMS movement after own settling [m] | 1.103 | 10.061 | 0.155 |
| Repeat maximum body-position difference [m] | 0 | 0 | 0.3411 |
| Instrumented wall time [s] | 34.465 | 142.521 | 191.197 |
| Instrumented real-time factor | 0.966 | 0.234 | 0.174 |
| Maximum connect-pin gap [mm] | 0.0693 | 0.00913 | 0.00412 |
| Maximum ID mass-conservation residual [kg] | 0 | 0 | 0 |

\* **Not an equivalent material/collision benchmark.** Feasibility measurements
are shown for diagnosis; they are ineligible for a matched physics ranking.
Mass and full body-frame inertia mappings were audited at runtime: maximum errors
1.53e-6 kg and 2.94e-6 relative, respectively. Initial maximum body position and
orientation differences were 4.95e-6 m and 1.03e-4 rad. These conversion checks
establish neither contact equivalence nor machine accuracy.

† Installed Newton 1.6.1 is available, but this experiment exercises its underlying
MuJoCo-Warp technology directly on the exact compiled reference model. Newton USD
import and Newton granular/MPM coupling are **not tested**. Do not call this an
independent Newton soil-model validation.

- Acquisition/contact/mouth observations here are all 20 Hz. Brief events can be
  missed; the original controller capture's 240 Hz funnel is a separate measurement.
  Reaction is the sum of bucket/grain force magnitudes, not net cylinder force.
  Wheel/ground signed XYZ reactions, all 19 joint coordinates (five cylinders),
  articulation and immutable actuator inputs are saved separately.
- Penetration is a documented +X, world-Y edge-strip proxy into the **common native
  settled sphere envelope** at lip height. An edge above/beside the envelope is
  unavailable, not zero. It is not chassis travel or current deformed soil depth.
- RMS pile movement uses each backend's state at 10 s to exclude release fall.
  PhysX already spread substantially during settling. Its absent rolling resistance
  is a plausible contributor; this experiment does not isolate the cause.
- MJWarp diverged during settling before pile attack. Its two full logs emitted
  28/37 solver iteration-limit warnings with the common 1000-iteration setting.
  Native warning counters copied to the host do not capture those GPU warnings.
  The repeats disagree despite identical inputs. Small connect gaps alone do not
  establish convergence or comparable mechanics. No convergence knob was tuned.
- Initial mass was 1435.296790 kg. Native final remaining/bucket/spilled/delivered/
  boundary-loss masses were 1332.988174/102.308616/0/0/0 kg; both alternatives
  retained all mass in the remaining/displaced categories. Displaced, never-acquired
  grains are **not spillage**. ID bookkeeping conservation does not validate contact
  forces, overlap or realistic pile shape. No run demonstrates measurable spillage;
  this benchmark does not complete M2 acceptance.
- Timing includes one small environment, synchronization and logging. The separate
  physics-loop times are 33.537/134.875/178.019 s (native/PhysX/MJWarp); these include
  stepping dispatch and, for PhysX, contact callbacks. Startup/JIT is recorded
  separately and caches were warm. This is not batched RL throughput evidence.

## Exact mapping limits

| Property | Native / direct MJWarp | PhysX feasibility adapter |
|---|---|---|
| Vehicle mass/inertia, source bucket pieces | Same compiled model | Same derived v1 USD; runtime mass/inertia audited |
| Joint effort and passive terms | Same tape; zero passive terms | Same tape; explicit zero armature, spring, damping and friction; no velocity clamp |
| Initial states / time | Common release, 240 Hz, eight substeps | Restored after Lab reset, before physics; same schedule |
| Sliding coefficients | Existing M2 law | 0.65 grain, 0.3 bucket, 1.0 tire/floor using min combine; this does not map the full law |
| Soft normal / rolling / torsional law | Same solref/solimp/condim | No equivalent mapping implemented; PhysX Coulomb/patch contacts differ |
| Collision surface/filtering | Analytical exported tire cylinders; native masks | Source tire convex hulls, 32 explicit body exclusions; full mask parity unverified |
| Contact force | Native engine contact force | Last-substep contact impulse divided by physics dt; reporting explicitly enabled |

The PhysX adapter stops on initial-state/inertia failures. It does not silently
substitute a position controller or inject material. Remaining numerical differences
include CPU double versus GPU single precision and different constraint solvers.
The same open-loop effort tape can amplify small mechanical deviations; this is a
useful diagnostic, not proof that a closed-loop controller would fail identically.

Validation: six complete replays (two per backend), 152 core tests and three USD
authoring tests passed. This validates reproduction/instrumentation, not WA320 accuracy.

## Recommendation

- **Diagnostic/reference:** retain native MuJoCo and all existing M2 code. It is
  repeatable here and has the most inspectable existing diagnostic chain, not proven
  real-world accuracy.
- **Primary RL backend:** defer selection. First establish unloaded mechanism/ground
  parity and convergence, then matched bulk-material and bucket tests. No PPO yet.
- **Newton/MJWarp:** useful as a same-law CPU/GPU cross-check; this run fails parity
  and repeatability. It adds no demonstrated granular constitutive fidelity. Newton
  MPM or alternative soil coupling needs a separate calibrated feasibility test.
- **PhysX:** retain as an independent validation candidate with its own explicitly
  identified contact law; current unmatched results cannot validate MuJoCo digging.
- **Real data before RL:** measured masses/COM/inertias and bucket/pin geometry;
  cylinder areas, pressure/stroke/velocity/force, relief and delays; tire slip and
  drawbar tests; material packing/repose and grain/steel friction; matched pile
  penetration force, bucket trajectory, payload retention and spillage histories.
  Backend-specific numerical convergence studies must precede physical calibration.

## Reproduction

Keep existing installations intact. Reference: Python 3.12, MuJoCo/MJWarp 3.12.0,
Isaac Lab 3.0.0, Newton 1.6.1, Warp 1.17.0. PhysX runner: existing full Isaac Sim
6.2.0.0 environment (Warp 1.19.0), appending the existing Lab site-packages path.
No ROS, legacy simulator migration or package upgrade is required.

Generate the passing v1 source with the existing instructions in
[reconstruction status](../calibration/WA320_RECONSTRUCTION_STATUS.md#implementation-and-reproduction).
Then, from the repository root (set `PYTHONPATH=src`):

```bash
.venv/bin/python scripts/prepare_backend_benchmark.py --source outputs/wa320/mechanical_correction/release_nominal/effective_model.xml
.venv/bin/python scripts/run_backend_benchmark.py --backend native --output outputs/wa320/backend_benchmark/native
.venv/bin/python scripts/run_backend_benchmark.py --backend native --output outputs/wa320/backend_benchmark/native_repeat
.venv/bin/python scripts/run_backend_benchmark.py --backend mjwarp --output outputs/wa320/backend_benchmark/mjwarp
.venv/bin/python scripts/run_backend_benchmark.py --backend mjwarp --output outputs/wa320/backend_benchmark/mjwarp_repeat
.local/komodo-isaacsim/bin/python scripts/run_physx_backend_benchmark.py --usd outputs/wa320/mechanical_correction/release_nominal/wa320_newton.usda --lab-site-packages .venv/lib/python3.12/site-packages --output outputs/wa320/backend_benchmark/physx
```

Run the last command once more with output `outputs/wa320/backend_benchmark/physx_repeat`.
Isaac Sim requires an already accepted NVIDIA EULA. Environment paths above refer
to this project's existing isolated installations; equivalent installations may be supplied.

```bash
.venv/bin/python scripts/evaluate_backend_benchmark.py
.venv/bin/python scripts/render_scripted_scoop.py outputs/wa320/backend_benchmark/bundle
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
```

Outputs remain ignored under `outputs/wa320/backend_benchmark/`: SHA-256-checked
model/effort bundle; per-run `states.npz`, final pile coordinates, `report.json`;
PhysX `mapping.json`; `comparison.json`, comparison/cylinder/pile/command figures;
and native reference replay `bundle/scripted_scoop_replay.mp4`. The comparison
refuses different bundle hashes, incomplete runs or unequal sample times.
`--max-ticks` is smoke testing only and explicitly produces an incomplete result.
