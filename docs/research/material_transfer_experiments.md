# Material transfer experiments and implementation order

Research hypothesis: a world model conditioned on granular material properties can
reduce the new-material data needed for useful wheel-loader loading predictions.
This is untested. First scope: dry, non-cohesive sand, crushed gravel, loose coal,
and subsequently graded mixtures. No training or new simulator is run in this task.

## Experimental matrix

| ID | Training domain | Held-out test | Main question |
|---|---|---|---|
| A | Sand property variations | Unseen sand property combinations/lots | Interpolation versus unseen combinations |
| B | Sand only, no material conditioning | Dry coal, no coal labels | Unconditioned zero-shot baseline |
| C | Sand variations with material descriptors | Dry coal, no coal labels | Does property conditioning improve extrapolation? |
| D | Sand-pretrained shared model + limited coal cycles | Disjoint coal lots/conditions | Target-data efficiency of adaptation |
| E | Multiple granular materials | Unseen mixtures and fractions/gradings | Composition generalization |
| F | Multiple numerical resolutions of the same physical samples | Held-out resolution | Robustness to numerical representation |

For each row, evaluate the same four model families where scientifically possible:

1. Material-specific models (target-trained oracle curves / scratch-data baseline).
2. Shared backbone without material conditioning.
3. Shared backbone with continuous material descriptors and validity masks.
4. Shared pretrained backbone with few-shot adaptation.

A target-trained coal oracle is **not zero-shot**; report its target-data cost.
A few-shot model at budget zero collapses to its pretrained predictor. For rows B/C,
show zero-shot results separately from positive-budget adaptation/oracle results.
With one fixed sand descriptor, C cannot identify how outcomes vary with material
properties. Train sand property variations and report whether coal lies outside
that support; conditioning alone cannot guarantee extrapolation. Add a leave-one-
material-out multi-material variant after B/C, without relabelling it sand-only.

## Controls and splits

- Fix WA320 asset, bucket, initial pose, controller/action semantics, pile boundaries,
  preparation, environment and output event definitions for controlled comparisons.
  Different physical materials may settle into different shapes; record the measured
  initial surface and packing rather than pretending identical releases stay identical.
- Compare models on the same immutable initial-state/action records. Across controllers,
  replay the same prepared simulator state; across backends use the common release and
  document incompatible laws as in the current backend benchmark.
- Use matched physical pile envelopes, preparation and initial-state distributions;
  report mass/packing differences when bulk density changes. Do not equalize mass by
  silently changing particle density or bucket geometry.
- Split by material-property combination/lot AND whole sequential rollout/seed/pile
  ancestry. No adjacent cycles, resampled heightmaps, mirrored variants or resolution
  replicas of a held-out trajectory in training. Keep mixture fraction intervals and
  physical grading families held out, not random points from the same trajectories.
- Fit scalers, embeddings, imputation and feature selection on training only. Hold
  calibration data separate from physical validation and from model evaluation.
- For F, physical sample/grading/geometry stays fixed; only numerical resolution
  changes. Calibrate each law using the same calibration protocol, freeze it before
  testing, and disclose residual bias. A finer DEM run is a sim-to-sim reference, not
  physical ground truth. Rebuild/settle conservatively; never resize live particles
  without mass/momentum accounting.
- Record resolution/backend hashes for stratified analysis. Do not mix failed PhysX
  mapping or unconverged MJWarp runs into ground-truth labels. Preserve failure rates;
  do not silently discard hard samples to improve prediction errors.

## Small, preregistered pilot budgets (proposed, not paper results)

After physical/solver gates pass, begin with 3 independent preparation seeds × 4
bounded controller settings = 12 scoops per material/property configuration. Repeat
one exact reset/command case three times first. This is a feasibility/variance pilot,
not enough evidence for a high-capacity world model. Select a small physically
measured property panel; do not sweep invented coal/sand constants.

- Record proposed learning curves at 32/128/512 source cycles only if the pilot
  justifies collecting them. Do not launch these batches now.
- Target adaptation budgets: 0/4/16/64 complete coal cycles; target training,
  validation and test rollouts remain disjoint. Use the same target cycles for all
  competing models at each budget; include scratch and full fine-tuning controls.
- Repeat model fitting with at least three random seeds when learning is authorized;
  report per-domain mean and confidence intervals clustered by physical lot/rollout.
  Include a mean/linear or small regression predictor before a large shared network.
- Predefine acceptable errors from sensor uncertainty and the loading decision's
  tolerance before training; do not choose thresholds after seeing test performance.
  No numerical real-world accuracy threshold is asserted without those data.

## Validation and reporting

| Level | Required measurements / reporting |
|---|---|
| Material | Bulk density, measured packing, static/dynamic repose, flow/runout and shear response with apparatus, uncertainty and held-out preparations |
| Digging | Actual cutting-edge penetration; bucket reaction histories; velocity field where measurable; acquired/retained/spilled mass; actuator work; before/after pile surface and conservation |
| World model | Payload MAE [kg], time MAE [s], work MAE [J], force MAE/peak error [N]; height MAE/RMSE [m], absolute surface-volume error [m³], mass/volume consistency; uncertainty coverage |
| Sequential prediction | One-step teacher-forced and free-running errors at 1/5/10/20/40 cycles; identical dig choices, cumulative mass/work/time errors, surface drift and repose violations |
| Eventual decisions | Compare world-model-selected actions to a bounded scripted search on the same held-out physical cases: retained mass, work, time, spillage and stability |

Relative errors near zero payload/work need a prespecified denominator floor and
an absolute-error report. A force MAE, DTW path score and aggregate error are distinct.
Report mechanical net/positive actuator work separately; no inferred fuel savings.
Heightmap surface volume is not automatically material mass when packing changes.

Every experiment report must include:

- Actual source/target/calibration/validation/test simulation counts and unique
  preparation/lot counts; attempted, failed and excluded runs with reasons.
- Prediction and generalization errors by held-out combination and material, not
  only a pooled average. Simulator-label agreement and real-data agreement separate.
- Minimum adaptation cycles needed to meet the preregistered tolerance, with
  matched scratch learning curves; no data-saving claim when tolerance is unmet.
- Physics wall time/RTF, preparation/settling, memory, training and inference cost,
  amortization assumptions, and number of physical tests. Keep compute and data costs
  distinct; report startup/cache/instrumentation conventions.

**Current results: none for A–F.** Existing backend comparisons and six-particle
payload are diagnostics, not transferable world-model or soil-validation evidence.

## Prioritized implementation roadmap

| Order | Work | Exit gate / reason |
|---|---|---|
| 0 — this task | Verify papers; version material/resolution records; propose transition/interface/splits | Page-specific audit, explicit unknowns, parser tests; no physics changes |
| 1 | Resolve unloaded mechanics/contact convergence and force instrumentation | Stable, repeatable reference with trustworthy loads; prevents learning numerical failures |
| 2 | Characterize reference gravel using existing heap/column/blade tools | Identified grading/preparation, matched bulk/flow/shear and held-out tool forces; strongest available full-loader literature reference |
| 3 | Validate bounded WA320 scripted digging and conservative material accounting | Physical acquisition/retention/spillage, measured work and surveyed surface with stated uncertainty; obtain field data, not just animations |
| 4 | Characterize dry sand, then loose dry coal with the same apparatus | Sample-specific held-out validation; coal fragmentation/porosity and dry scope recorded |
| 5 | Add a transition exporter using the proposed interface; freeze split manifests | Valid frames/units/masks, data lineage, no leakage, complete solver/material validity gates |
| 6 | Run small A/B/C predictive baselines, then D learning curves | Shared-conditioning benefit and target-data savings measured against fair controls |
| 7 | Investigate conservative multiscale coupling and E/F | Justify complexity by calibrated resolution costs and mixture needs; separate PSD and representation effects |
| Later | Test loading decisions, then consider RL/world-model planning | Accepted physics and predictive/decision gates; no PPO or planning training authorized here |

Aoshima's multiscale result motivates investigation, not a commitment to a new engine.
Do not delay independent material/tool calibration while building a full G-model.
The predictive-research ordering above is a proposal; activating learning still
requires an explicit milestone decision about the canonical M5/M6 gates. This task
does not change those gates or enable training.

Follow-up execution belongs in the M2/M3 issue tracker; the table states dependencies,
not dates or claims of completed simulator/training milestones.
