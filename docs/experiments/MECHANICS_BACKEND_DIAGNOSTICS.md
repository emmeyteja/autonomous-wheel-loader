# Unloaded mechanics and solver diagnostics

- Purpose: isolate mechanism/contact differences before material calibration or backend selection. **Sim-to-sim only; no measured WA320 accuracy validation.**
- Preserve the original [M2 benchmark](PHYSICS_BACKEND_BENCHMARK.md), v0/v1 assets, contact parameters and effort tape. No PPO, payload fitting or granular scaling.
- Runtime: MuJoCo/MJWarp 3.12.0. Direct MJWarp tests do not validate Newton USD import or granular coupling.

## Controlled fixtures

- `fixed_no_contact`: remove grains, mount the rear chassis, disable all contacts; retain the mechanism, gravity and original actuator efforts.
- `unloaded_ground`: remove grains; retain the floating vehicle, floor and original efforts.
- Both replay the first 3 s of the unchanged nine-channel effort tape at 240 Hz. This interval is the settling phase, not a scoop.
- Initial vehicle body-position discrepancy from the parent release is zero. Audited vehicle masses, inertias, collider geometry, equality parameters, actuator parameters and contact arrays have zero differences. Preparation rejects changed audited values or release positions.
- The authoritative compiled parent model preserves sphere dimensions/masses. XML is an inspection/replay aid and can round values; it is not the authoritative physics input.
- Log state, joint velocity, applied actuator force, generalized constraint force, contacts, solver iterations and pin gaps at 240 Hz. The original full benchmark logged at 20 Hz. Logging cadence is matched within these diagnostics; cadence invariance has not been established.
- Baseline: CG, iteration cap 1000, 8 substeps/control tick (1/1920 s), tolerance 1e-6. Variants change only the stated numerical setting in both engines: cap 2000; 16 substeps; or Newton solver.

## Quantitative findings

Maximum / RMS native-to-MJWarp bucket-origin discrepancy over 3 s:

- Fixed, CG: **17.995 / 6.139 mm**.
- Fixed, CG cap 2000: **17.995 / 6.139 mm**.
- Fixed, CG 16 substeps: **24.336 / 13.173 mm**.
- Fixed, Newton: **0.347 / 0.041 mm**.
- Ground, CG: **73.015 / 25.467 mm**.
- Ground, CG cap 2000: **52.001 / 20.136 mm**.
- Ground, CG 16 substeps: **75.829 / 30.797 mm**.
- Ground, Newton: **0.056 / 0.019 mm**.

These are observed discrepancies, not convergence guarantees or accuracy tolerances. The 1 mm timestamp emitted by the evaluator is a visibility threshold, not an acceptance criterion. Doubling the iteration cap or substeps did not establish convergence. Sampled CG iteration maxima were 21–108, below either cap; the ground cap variant must not be credited with an improvement without considering repeat variability. Newton used at most two sampled iterations in these short fixtures.

- Repeat each fixed/ground CG/Newton fixture once using the identical bundle: native maximum body-position difference is zero in all four cases. MJWarp is zero except ground/Newton, **0.02681 mm**. This is short-fixture repeatability, not full-scoop repeatability.
- Repeated MJWarp runs report zero final-step overflow flags. Earlier variant runs did not record that field; none provide a transient-overflow audit.

Native-only ground sensitivity to CG tolerance:

- Native CG versus native Newton: maximum bucket discrepancy **0.149 mm**.
- Native CG at tolerance 1e-10 versus native Newton: **0.00154 mm**.
- Installed MJWarp clamps tolerance to at least 1e-6; this tighter-tolerance study is **not a matched GPU comparison**.
- Small constraint gaps alone do not prove trajectory equivalence or physical accuracy.

### Open-loop sensitivity before digging

- Replay the full 83-sphere scene for 10 s using native CG, with identical efforts, material and geometry.
- Change only the initial curl-cylinder velocity by **1e-7 m/s**.
- Maximum bucket-origin difference: **0.52347 m**. Final bucket-origin height changes from **0.51657 m** to **0.98372 m**, before pile attack.
- This deliberately perturbed run is a sensitivity test, not a matched-backend comparison. It demonstrates amplification under the open-loop effort tape; it does not identify a physical machine instability.
- The earlier 102.309 kg retained by native CG therefore cannot establish superior digging fidelity. Solver-dependent motion can change the attack itself before material contact.

### Additional scope and incomplete runs

- PhysX unloaded-ground feasibility run completed: maximum bucket discrepancy versus native CG **0.24699 m**, maximum connect-pin gap **0.00899 mm**. Existing cooked tire geometry and non-equivalent contact laws remain; no matched physics ranking is justified. The fixed/contact-disabled PhysX fixture is explicitly unsupported.
- A full 33.3083 s native Newton replay completed with **0 kg retained** and peak sampled bucket/grain reaction **53.358 kN**. This is a separate solver experiment, not an improvement or a validated scoop.
- Its matching full-pile MJWarp process was interrupted before completion and produced no completed report. Exclude it from comparison; do not infer its payload or full-cycle repeatability.
- Instrumented runtimes are saved locally, but overlapping jobs and changed observation cadence make this study unsuitable for throughput ranking.
- GPU capacity metadata covers the final step only. A zero final overflow flag is not an audit of all transient capacities.

## Reproduction

Run from the repository root in the pinned simulation environment. First create the preserved parent bundle following [the backend benchmark instructions](PHYSICS_BACKEND_BENCHMARK.md). Outputs remain ignored/local.

```bash
PYTHONPATH=src .venv/bin/python scripts/prepare_mechanics_benchmark.py \
  --case unloaded_ground --output outputs/wa320/mechanics_benchmark/ground/bundle
PYTHONPATH=src .venv/bin/python scripts/prepare_mechanics_benchmark.py \
  --case fixed_no_contact --output outputs/wa320/mechanics_benchmark/fixed/bundle
# Repeat each fixture with --solver Newton and a separate *_newton output.
# Optional independent studies: --iterations 2000 or --substeps 16.
# Native-only tolerance study: --tolerance 1e-10.
PYTHONPATH=src .venv/bin/python scripts/run_backend_benchmark.py \
  --bundle outputs/wa320/mechanics_benchmark/ground/bundle \
  --backend native --output outputs/wa320/mechanics_benchmark/ground/native
# Run the same bundle with --backend mjwarp and a separate mjwarp output.
# Repeat identical bundles into native_repeat / mjwarp_repeat directories.
PYTHONPATH=src .venv/bin/python scripts/evaluate_mechanics_benchmark.py
PYTHONPATH=src .venv/bin/python scripts/render_mechanics_benchmark.py
```

- Populate both fixed/ground and their Newton variants before rendering the four-panel replay. Rendering replays recorded qpos with forward kinematics only; it never steps or corrects physics.
- Evaluation writes `comparison.json` and `mechanics_comparison.png`, verifies bundle hashes/time alignment and reports available repeat differences. Missing full-pile reports are excluded.
- Sensitivity: prepare `pile_reference --seconds 10` into `settle_cg/bundle`; prepare an otherwise identical `settle_perturbed/bundle` with `--curl-velocity-perturbation 1e-7`. Replay each with native into its `native` directory. No particle teleportation or assigned payload.
- Full-duration variant: `pile_reference --seconds 33.30833333333333 --solver Newton`, separate `pile_newton/bundle`; a completed native run alone is insufficient for a cross-backend row.
- Regression checks: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest` (165 passed) and the same command with `tests/integration/test_mechanics_benchmark.py` (3 passed). Disable unrelated external pytest plugins for the core suite. The latter checks fixture preparation, release preservation, exact particle parameters and empty-material accounting; synthetic fixtures are software tests, not physics validation.

## Recommendation and acceptance

- Follow-up [constraint-consistent investigation](MECHANISM_SENSITIVITY_INVESTIGATION.md) checks the original invalid isolated-slider velocity, repeats corrected tangent perturbations and audits full-duration Newton CPU/GPU replay. Use that investigation for the updated reliability decision.

- Keep native MuJoCo and the original open-loop replay as diagnostic references. The short Newton results support further solver investigation, not a primary RL backend decision.
- Next implement a **separately labelled common bounded feedback/intent-command benchmark**, using the same controller law, gains and limits across engines; record the resulting physical efforts. Keep the strict identical-effort experiment alongside it. Do not tune controllers independently to obtain a desired payload.
- Establish repeated unloaded trajectory/contact behavior before fitting bulk density, repose, flow or digging forces. Material calibration against gravel and dry sand remains pending.
- M2 remains incomplete: no new validated retained-payload/spillage result, no real WA320 force/trajectory calibration, and no accepted granular or RL backend.
