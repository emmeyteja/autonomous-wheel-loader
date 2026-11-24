# M2 deterministic scripted scoop: reservoir and acquisition study

## Status and scope

This is an **uncalibrated coarse-grain CPU experiment**, not calibrated sand,
validated soil mechanics, real WA320 operating guidance, or Isaac Lab/Newton GPU
material execution. M2 remains incomplete while any required physical gate fails.
No PPO, world-model training or production GPU granular backend is enabled.
Work belongs to [M2 issue #19](https://github.com/emmeyteja/autonomous-wheel-loader/issues/19).

The unchanged `WA320_reconstructed_v1` vehicle comes from the passing mechanical
source at `97ae4e5`. The source parameter digest and compiled 15,175 kg vehicle
mass are checked. The active terrain default remains rigid. Native MuJoCo 3.12.0
advances the complete Newton-exported loader and independent free rigid spheres
in one frictional dynamics solve. Grains are never inserted into the bucket,
attached, teleported, deleted on contact or assigned as vehicle ballast.

## What the old 102 kg result actually contains

`36418cb` had a **1,500-particle budget**, not 1,500 simulated particles. Its nominal
pile contains **83 spheres**, initially **1,435.297 kg**; six retained spheres weigh
**102.309 kg**. `small_debug` preserves that exact geometry, radius distribution,
seed and controller. The full measured state trajectory and retained mass are
compared with the old recording separately from the new research-pile repeat.

At 240 Hz observation the baseline funnel is:

- Initial: 83 particles, 1,435.297 kg.
- Contacted by bucket: 17 particles, 290.844 kg.
- Entered the mouth and acquired: six particles, 102.309 kg.
- Retained at breakout and finally retained: six particles, 102.309 kg.
- Physically observed mouth spillage: zero.
- Never acquired, displaced sideways by more than one diameter: 18 particles,
  307.811 kg. Of these definitions, being outside the source's lateral bounds is
  a different measure: ten particles, 171.865 kg.
- Never acquired and still inside the original settled source bounding region:
  31 particles, 539.098 kg. Remaining material outside that region is still
  accounted for; it is not spillage.

The six retained masses are 17.1785, 16.3419, 16.4970, 17.6206, 16.5383 and
18.1324 kg. They are observations, not assigned payload values. A 20 Hz replay
finds only 16 contacted particles (274.006 kg); the finer live observation catches
one more brief contact without changing dynamics or the six acquired IDs.

The measured baseline pile envelope is about 1.211 m³. The discretized bucket
cavity sweep is about 9.35 m³: **sweep/pile ≈ 7.72**. The source does not extend
past both bucket sides (minimum lateral margin −0.607 m). This is an inadequate
general reservoir experiment. It does not prove that insufficient material is
the sole cause of the six-particle result.

## Pile design and physical mass

`configs/experiments/scripted_scoop.yaml` configures width, depth, height, crest
length, release slope, solid density, target bulk density and target packing.
These are **research preparation dimensions, not claims about a WA320 site**.
The release is a trapezoid extruded across its width; its lateral faces are free,
with no retaining walls. Gravity can spread the released material in all directions.
Release slope is not an imposed or validated angle of repose.

For height h, crest c, width w and symmetric front/rear release slope θ:

```
depth d = c + 2h / tan(θ)
pile preparation volume V = w h (d + c) / 2
particle mass_i = solid_density × (4π/3) × radius_i³
target material mass = V × target_bulk_density
N × mean_actual_particle_mass = actual_compiled_material_mass
bulk_density = material_mass / measured_bulk_envelope_volume
packing_fraction = bulk_density / solid_density
```

Depth, crest, height and slope are geometrically coupled; inconsistent inputs fail
instead of silently changing the shape. For volume-designed cases the solid
trial density is 2,590 kg/m³, release packing target 0.4 and release bulk-density
target 1,036 kg/m³. These are numerical research assumptions, not sand calibration.

Count is derived from target mass and the expected mass of the unchanged uniform
radius distribution [0.95r, r]. A deterministic selection of non-overlapping lattice
positions then matches the target mass within half the largest element mass.
No density inflation, radius shrinkage or truncation to `max_particles` occurs.
The guard rejects requests beyond the CPU count budget or non-overlapping release
capacity. `small_debug`, `low_shallow`, its width ablation and `tall_deep` retain
the original complete-lattice preparation for exact historical comparisons; their
actual mass and implied density are measured rather than claimed to match a new
bulk-density target.

Research preparation (both `nominal` and the default `research_underburden`): **3.8 m wide × 5.0 m deep × 1.2 m high**,
crest 1.0 m, slope 30.9638°, 13.68 m³, **816 elements**, actual mass 14,179.665 kg
(target 14,172.480 kg). Requested dimensions do not imply exact settled dimensions.
The authoritative settled initial mass is the sum of actual compiled element
masses after a valid settled state, not the requested target mass. The default
`research_underburden` explicitly raises the lip-height plan to 0.30 m, enough
for a coarse element beneath the cut. `nominal` preserves the old 0.045 m plan
for the unchanged-controller study. This is a geometric underburden correction,
not a payload or force-threshold adjustment.

At r = 0.12 m, a 3–4 m × 3–5 m × 1–1.5 m pile naturally needs roughly a thousand
coarse elements, not tens of thousands. The performance cases therefore enlarge
the physical reservoir, keeping element size and material/controller settings:

- `debug_1500`: 4 × 5 × 1.5 m, crest 3.25 m, release slope 59.7436°,
  24.75 m³, approximately 1,500 elements. This steep *release* must settle freely;
  it is not a claimed stable sand slope.
- `medium`: 6 × 10 × 2.4 m, crest 2 m, slope 30.9638°, 86.4 m³,
  approximately 5,000 elements and 89.5 tonnes.
- `large`: 8 × 12 × 3 m, crest 2 m, slope 30.9638°, 168 m³,
  approximately 10,000 elements and 174 tonnes.

The `small_debug` safety budget is 1,500 but its historical particle count remains
83. Performance targets must not be confused with this historical debug state.

The 0.12 m nominal radius means **0.24 m nominal diameter**. These are coarse
numerical elements, not individual sand grains. The shallow source is fewer than
two nominal diameters high after settling. No fine-sand fidelity is inferred.

Contact trials remain unchanged: grain sliding friction 0.65; bucket/steel sliding
friction 0.3; grain torsional/rolling length coefficients 0.0096/0.0288 m; steel
coefficients 0.0096/0.0096 m; 4 ms contact time, damping ratio 1 and six-dimensional
contact. The length coefficients retain the earlier radius scaling from the column
diagnostic; none is identified against real sand or steel tests. The native solve
uses implicit-fast integration, CG with a 1,000-iteration cap, 240 Hz control and
eight physical substeps per control tick. These numerical choices are not a material
calibration or independent validation of digging resistance.

## Measured geometry, capacity and boundaries

`initial_pile_envelope` uses the convex hull of 26 sampled surface directions per
sphere. Its approximate bulk volume includes interparticle voids and concavities;
it is not a sum of sphere solid volumes. Effective settled bulk density and
packing are mass divided by this measured envelope. A budget interruption before
settling is marked **UNSETTLED** and cannot establish authoritative settled mass.

`bucket_swept_volume_m3` is a 0.10 m voxel union of centroid-classifier cavity
poses sampled during penetration/curl advance. It measures an approximate cavity
sweep, **not actual excavated solids**. The report gives sweep/pile and its inverse,
plus global-envelope lateral clearance, floor-to-edge clearance and distance ahead of the
last penetration pose. These bounding-envelope margins do not establish local material
occupancy beneath or around every cutting-edge position; that remains unvalidated. An early force-triggered scoop has a shorter measured sweep
than the requested full-depth path; do not use its ratio to claim full-depth success.

Nominal bucket capacity is approximately 3.0 m³. Report separately:

```
solid_material_volume = retained_mass / solid_density
bulk_equivalent_retained_volume = retained_mass / measured_initial_bulk_density
bulk_equivalent_fill_factor = bulk_equivalent_retained_volume / 3.0 m³
```

The bulk-equivalent quantity assumes the initial-pile effective density as a
reference; it does not measure packing inside the bucket. Baseline effective
bulk density is about 1,185.23 kg/m³, so 102.309 kg is about 0.0863 m³ or **2.88%**
bulk-equivalent fill, versus **1.317%** solid-volume/nominal-capacity ratio.
The unchanged estimated bucket profile extrudes to about **4.30 m³ before plate
clearance**, unlike the 3.0 m³ nominal specification. This geometry uncertainty is
reported, not corrected to force a payload or declared a validated bucket cavity.

## Contact, acquisition, retention and spillage

All IDs and compiled masses are fixed. `AcquisitionFunnel` observes bucket-local
centres at 240 Hz, independently of 20 Hz state recording:

1. **Contact:** a grain/bucket pair appears in the solver contact list. This never
   assigns payload. Contact can be missed between observation ticks.
2. **Mouth entry:** the measured centre segment crosses the finite open mouth
   plane inward, within the lip/back-top segment and bucket lateral aperture.
3. **Acquired:** a mouth-entry ID subsequently satisfies the cavity classifier.
4. **Retained after breakout:** classified inside on the first reverse observation.
5. **Physical spillage during extraction:** a previously acquired centre crosses
   the finite mouth outward during reverse, optional tip/hold or stabilization.
6. **Final payload:** centres still inside after the measured sequence. Support,
   height and final-speed screens are reported separately.

The classifier uses the original extruded profile with half-plate-thickness
measurement clearance. Whole-element mass is counted for an included centre;
partial protrusion is possible. The mouth plane is a measurement surface, not a
collision lid. Unverified inside appearances and wall-only classifier exits are
exposed, not used to satisfy physical-spillage acceptance. Entry/exit event times,
IDs, phases and masses are recorded. Acquisition and spill counts are unique IDs;
repeated events are also counted separately. Contact > acquired > retained is
possible, not an inequality forcibly imposed on results.

The exclusive instantaneous mass partition remains:

```
initial = never_acquired_remaining + bucket + acquired_currently_outside
          + delivered + boundary_loss + residual
```

The older `spilled_mass_kg`/`bucket_exit_mass_kg` channels mean acquired IDs
currently outside the *centroid classifier*. Physical mouth spillage has its own
funnel field and acceptance gate. Displaced never-acquired material is never
labelled spillage. Delivery remains zero in this scoop-only experiment.

## Controller and force interpretation

The controller uses bounded drive and physical cylinder efforts only:
settle → approach → align → lower → penetrate → curl/advance → lift/breakout →
reverse → stabilize. Penetration target is 1.7 m, attack −5°, curl starts at 90%
of target depth, and the existing reaction-force trigger remains 20 kN.

The original trigger uses **sum of grain-contact force magnitudes at logging
ticks**, not the net vector resultant. New runs additionally record the signed
world XYZ grain-reaction resultant. A force trigger can end penetration and then
end curl/advance one control tick later; this behavior is measured before any
attempt to change controller policy or increase its threshold. Cylinder force,
stroke, pin closure, wheel slip, traction, stability, contact overlap and solver
warnings remain acceptance screens. Actuator work is mechanical effort × joint
velocity, not fuel or engine energy.

`spill_demonstration` reuses the exact small-debug acquisition, lifts, reverses,
then follows the existing reachable dump linkage plan through physical cylinders
and holds it. No grain state or payload mass is commanded. If all material exits,
the physical-spillage gate may pass while the final-positive-retention screen
fails; that is reported rather than hidden.

## Shallow-case investigation

Keep `low_shallow` and its positive-retention acceptance test. It is a physically
failed scoop at the present coarse resolution, not a successful zero-payload test.
The historical recording completes its motion and has a reachable attack plan,
yet both the plate-clearance classifier and an independent zero-clearance polygon
check find **zero inside centres throughout**. Thus the plate-clearance exclusion
alone does not explain the zero payload. The low source has only 36 elements,
577.466 kg and about 0.427 m settled height; contact and displacement occur without
observed mouth entry. `low_shallow_wide` changes only source width to investigate
lateral supply, retaining low-case density, attack, depth, seed, radius and controller.
This does not identify fine-grain resolution, cutting-edge geometry or friction
independently. A failed physical explanation is not replaced by tuning to a payload.

## Reproduce

Install the existing `material-audit`, `geometry-audit` and `dev` extras. The
optional material diagnostics now also require SciPy for convex-envelope volume.
Create the passing v1 mechanical source first; do not substitute a fixed-base or
pre-correction model.

```bash
PYTHONPATH=src .venv/bin/python scripts/run_scripted_scoop_suite.py \
  --source outputs/wa320/mechanical_correction/release_nominal/effective_model.xml \
  --output outputs/wa320/m2/reservoir_suite \
  --research-wall-s 900 --benchmark-wall-s 120

PYTHONPATH=src .venv/bin/python scripts/check_scripted_scoop.py \
  --source outputs/wa320/mechanical_correction/release_nominal/effective_model.xml \
  --scenario research_underburden --output outputs/wa320/m2/reservoir_suite/research_underburden

PYTHONPATH=src .venv/bin/python scripts/evaluate_scripted_scoop.py \
  outputs/wa320/m2/reservoir_suite
PYTHONPATH=src .venv/bin/python scripts/diagnose_scripted_scoop.py \
  outputs/wa320/m2/reservoir_suite/low_shallow
PYTHONPATH=src .venv/bin/python scripts/render_scripted_scoop.py \
  outputs/wa320/m2/reservoir_suite/spill_demonstration
```

Each failing physical run exits nonzero but retains its report/recording. The suite
continues across failures and compares small/medium/large under identical vehicle,
controller, seed, radius, solid-density and contact settings. Historical packing
and geometry differ and are reported explicitly. `--reuse-existing` is optional;
the evaluator still checks each report against the requested physical scenario.

CPU wall budgets include preparation/compilation and stop with an explicit
incomplete-scoop reason. An external per-process timeout handles an individual
solver step or compilation that cannot return within the budget. Preparation and
runtime checkpoints survive such a timeout. The 512 MiB solver arena is bounded;
process peak RSS, contacts, warnings and simulated-seconds/wall-second are measured.
A short startup/settling benchmark does **not** provide final payload, stability
through excavation, or a completed pile-size sensitivity comparison.

## Artifacts and acceptance

All local experiment evidence stays under ignored `outputs/`. Each report contains
requested dimensions, count/radius/masses, measured envelope/density/packing,
sweep and boundary ratios, contact/acquisition/spill funnel, force/slip/stroke
metrics, conservation and explicit failed checks. `comparison.csv` compares the
measured cases; incomplete rows are labelled. `suite.json` preserves missing cases,
configuration mismatches, incomplete runs and failed shallow/spillage gates.

Recordings contain all free-body poses and fixed material IDs, lip positions,
accounting and actuator/contact traces. Replays never advance physics and assert
that highlighted payload matches the recorded mass. Videos are **recorded CPU
replay**, not live simulation or evidence of physical material calibration.

## Measured interpretation before controller changes

The original result is an **entry limitation**, not loss after acquisition: all
six mouth-entry IDs remain retained, with zero physical exits. The 83-element
source is too narrow and too small relative to the measured cavity sweep. Neither
the 3.0 m³ nominal capacity nor contact count predicts payload. Source geometry,
late curl (90% penetration), coarse resolution and blunt contact geometry remain
coupled; this study does not uniquely isolate wall/lip geometry or friction.
Inspection confirms separate side-wall meshes and floor/back plates, with no
full-width convex bucket hull closing the mouth.

Widening only the configured low source from 1.0 to 3.8 m changes 36 elements
(577.466 kg) to 153 elements (2,466.278 kg), and changes retention from zero to
seven particles (113.681 kg). It retains the low-case density, seed, radius,
controller, attack and depth. Its generated arrangement necessarily changes with
geometry; it is not an identical-particle-layout experiment. All single-run
physical checks pass for `low_shallow_wide`. The original failed case remains in
the suite and its acquisition gate remains active. This is the measured shallow
correction, without declaring the old zero-payload run a pass.

In the 816-element old-height research case, the unchanged 20 kN contact-magnitude
trigger sees 30.240 kN and ends penetration at 0.466 m chassis travel. It ends
curl/advance one control tick later, using the same sampled reaction, before any
mouth entry. With the default 0.30 m height plan, the edge is
0.277 m above the initial material envelope floor, minimum lateral margin 1.182 m, and material ahead of the final edge
3.803 m. The settled envelope is approximately 5.511 m wide × 4.287 m deep ×
0.987 m high, 12.261 m³, effective density 1,156.44 kg/m³ and packing 0.4465.
Sweep/pile is 0.5423 (inverse 1.844). Despite the larger global reservoir and floor clearance, a 21.426 kN reaction again triggers early breakout at 0.915 m chassis travel;
no grain enters. These short sweeps do not establish full-depth excavation.

The next controller investigation is the force-triggered transition and only
one tick of curl/advance, with the current 20 Hz reaction observation. Increasing
the force threshold or tuning toward a desired payload is not justified by these
results. The current study preserves that failure and does not claim successful
large-reservoir scooping.

The explicit tip-out run acquires six particles/102.309 kg, retains them through
lift and reverse, then records all six outward mouth crossings at about 38.800 s.
It is deliberate complete release, not spontaneous withdrawal spillage or a
partial-spill retention result. Mass remains accounted for; no grain is deleted.
The physical-spillage gate passes, while final-positive-retention/support checks
fail because the bucket ends empty.

<!-- measured-results -->
## Recorded results

Values below come from the recorded reference runs, not a target payload. All
mass columns are kg. Complete motion can still fail acquisition/retention gates.
A blank final payload for an interrupted test is unknown, not zero.

| Case | N | Settled initial mass | Contacted | Acquired | Physically spilled | Final retained | Sweep/pile |
|---|---:|---:|---:|---:|---:|---:|---:|
| `small_debug` | 83 | 1435.297 | 290.844 | 102.309 | 0.000 | 102.309 | 7.721 |
| `nominal` | 816 | 14179.665 | 192.499 | 0.000 | 0.000 | 0.000 | 0.413 |
| `research_underburden` | 816 | 14179.665 | 158.164 | 0.000 | 0.000 | 0.000 | 0.542 |
| `low_shallow` | 36 | 577.466 | 111.619 | 0.000 | 0.000 | 0.000 | 21.774 |
| `low_shallow_wide` | 153 | 2466.278 | 464.635 | 113.681 | 0.000 | 113.681 | 4.883 |
| `tall_deep` | 111 | 2019.370 | 325.555 | 129.937 | 0.000 | 129.937 | 6.524 |
| `spill_demonstration` | 83 | 1435.297 | 290.844 | 102.309 | 102.309 | 0.000 | 7.721 |

Bucket reaction below is the existing sampled sum-of-contact-magnitudes proxy,
not the signed vector resultant. Cylinder values are steer/lift/bucket channels.
Displaced mass includes acquired material; it measures material movement rather
than only rearrangement of the remaining pile. Full traces are in the recordings.

| Case | Peak bucket reaction (kN) | Peak slip velocity (m/s) | Cylinder peaks (kN: steer/lift/bucket) | Mass moved > one diameter (kg) |
|---|---:|---:|---:|---:|
| `small_debug` | 12.39 | 0.01487 | 20.77/82.19/34.13 | 1435.297 |
| `nominal` | 30.24 | 0.01177 | 7.19/82.19/33.89 | 16.770 |
| `research_underburden` | 21.43 | 0.01155 | 8.80/82.19/38.54 | 32.999 |
| `low_shallow` | 7.00 | 0.01995 | 9.14/82.19/27.23 | 577.466 |
| `low_shallow_wide` | 18.57 | 0.01429 | 9.14/82.19/40.17 | 1950.935 |
| `tall_deep` | 16.33 | 0.01493 | 9.47/82.19/42.66 | 2019.370 |
| `spill_demonstration` | 12.39 | 0.02192 | 20.77/146.77/242.54 | 1435.297 |

### CPU feasibility limits

These are shared-workstation measurements, not isolated hardware benchmarks.
An underburden diagnostic overlapped some startup benchmarks. Configured wall
budgets include compilation. RSS is the worker's measured peak resident memory;
512 MiB is its configured solver arena, not total process memory.

| Case | N | Prepared mass (kg) | Simulated time (s) | Total wall time (s) | Peak RSS (MiB) | Outcome |
|---|---:|---:|---:|---:|---:|---|
| `debug_1500` | 1473 | 25637.350 | 4.0125 | 120.14 | 183.98 | Incomplete; CPU wall budget |
| `medium` | 5151 | 89506.514 | 0.6750 | 120.43 | 440.60 | Incomplete; CPU wall budget |
| `large` | 10010 | 174047.680 | Not established | 240 s watchdog | Not captured | Compilation did not return; process stopped |

The ~1.5k and ~5k runs stop before the 10 s settling period completes. No solver
warnings were recorded in their short observed intervals; that does **not** prove
excavation stability. The 10k request retains its complete prepared count and
174-tonne mass target, but compilation did not return before the external watchdog.
No final settled mass, payload, contact/acquisition outcome, full-cycle stability
or authoritative peak RSS is claimed for that case. The current CPU path is
impractical at this scale under these budgets. No GPU workaround was introduced.
The requested completed small/medium/large payload sensitivity is therefore
**not established**; incomplete values must not be interpreted as failed zero-load
scoops. `comparison.csv` explicitly labels partial and missing-final-report rows.

Original-control research repeat maximum generalized-position error: 0.0 (mixed SI joint coordinates).
Default underburden repeat maximum generalized-position error: 0.0.

`baseline_comparison.json` confirms the original 36418cb nominal and `small_debug`
have identical prepared/settled hashes, sample times and recorded generalized
positions, with zero payload difference. `spillage_prefix_comparison.json` confirms
the tip-out case shares the identical recorded acquisition/lift/reverse prefix
through 30.254 s before deliberately changing orientation.

M2 remains incomplete: the original shallow case and the larger research scoops
fail acquisition, the deliberate full release ends empty, and the large CPU
sensitivity cases are incomplete. Geometric adequacy, mass closure or a playable
replay is not material-model validation. The next controlled investigation should
address force-triggered curl/breakout behavior and distinguish trajectory/contact
geometry from particle-resolution effects; no learned controller is started.

## Force-transition controller investigation

The reservoir study identifies a specific controller defect: the same sampled
20 kN reaction ends `penetrate`, then ends `curl_advance` one 1/240 s tick later.
The latter transition does not require measured curl-cylinder movement. Reaction
feedback was also tied to the 20 Hz recording schedule.

Two opt-in changes isolate these effects without changing vehicle geometry,
material, force threshold, seed, attack pose, penetration target or actuator limits:

- `--reaction-feedback-hz 240` observes the previous physical step's bucket/grain
  force-magnitude sum at each control tick, independently of 20 Hz recording.
  It preserves the existing proxy, rather than silently substituting a resultant.
  `recording.npz` includes command time, reaction sample time and reaction value
  at 240 Hz; transition events include sample age. This is simulator contact
  feedback, not a calibrated load sensor.
- `--load-response stop_and_curl` latches a zero forward-speed request when the
  existing force threshold or maximum depth is reached. The finite wheel servo
  brakes the vehicle; the PI integrator resets on that latch. The bucket then
  curls through the original bounded cylinder controller until the measured curl
  length is within 6 mm of the existing carry target before lift/breakout.
  Force relief does not restart advance. No minimum timed fill or state override
  is used. The existing 25 s phase timeout and mechanical screens still apply.

The 20 kN value is now a drive-stop/curl transition in the opt-in policy, not a
hard upper bound on total contact force or a manufacturer load limit. Curl can
change pile reaction while chassis advance is stopped. Cylinder efforts remain
bounded by the unchanged physical actuator limits. This is a controller semantics
change, not evidence of identified hydraulic or material behavior.

Legacy response and recording-rate feedback remain the defaults so historical
runs can be reproduced. No controller variant is promoted to a validated M2 task.
The new comparison uses the same 816-element `research_underburden` preparation
as the failed legacy baseline. First test fresh feedback with legacy transitions;
then use the same feedback with measured-curl transitions. Check prepared and
settled-state hashes before attributing differences to controller behavior.

```bash
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python \
  scripts/check_scripted_scoop.py \
  --source outputs/wa320/mechanical_correction/release_nominal/effective_model.xml \
  --scenario research_underburden --reaction-feedback-hz 240 --load-response legacy \
  --wall-budget-s 1200 --output outputs/wa320/m2/curl_transition/fresh_only

PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python \
  scripts/check_scripted_scoop.py \
  --source outputs/wa320/mechanical_correction/release_nominal/effective_model.xml \
  --scenario research_underburden --reaction-feedback-hz 240 \
  --load-response stop_and_curl --wall-budget-s 1200 \
  --output outputs/wa320/m2/curl_transition/stop_and_curl

PYTHONPATH=src .venv/bin/python scripts/evaluate_scoop_controller_comparison.py \
  outputs/wa320/m2/curl_transition
```

Repeat the candidate into `stop_and_curl_repeat` using the identical command.
The evaluator checks identical vehicle/material/geometry/other controller values,
prepared/settled hashes, completed motion sequences and a required candidate repeat
trajectory. Failed acquisition is still reported separately. Its
`comparison_valid` flag means a controlled comparison, not a successful scoop.
The generated `controller_comparison.png` compares payload, measured curl stroke
lift extension and the control reaction proxy (legacy uses its 20 Hz log). M2 acceptance remains separate.

### Observed controller comparison

- The old 20 Hz legacy run spends 0.00417 s in curl/advance and retains 0 kg.
  It contacts nine elements / 158.164 kg.
- Fresh 240 Hz feedback with legacy transitions still spends 0.00417 s in
  curl/advance and retains 0 kg; four elements / 69.319 kg contact the bucket.
  Thus updating force more often does not fix the transition logic.
- Fresh feedback with `stop_and_curl` spends **3.90417 s** in curl/advance. The
  recorded curl-cylinder extension increases approximately **0.1286 m**, then
  the phase ends on the measured curl target. It contacts three elements /
  51.074 kg but still has zero mouth entries and zero acquired/retained mass.
  Cycle time is 33.208 s, including settling.
- Both new trials start with exactly the same prepared and settled hashes as
  the old research baseline. All 816 compiled grain masses are unchanged.
  The legacy small-debug compatibility run also preserves all recorded joint
  positions, times and 102.309 kg payload exactly.
- First force-trigger observation is 22.208 kN at 17.2917 s. Only one 240 Hz
  observation exceeds 20 kN in the stop-and-curl run; the legacy-transition
  fresh-feedback run has two. These are sampled observations, not continuous
  impulse durations or validated soil-force measurements. The ordinary 20 Hz
  contact plot misses this peak, so the comparison also plots control-rate data.
- The stop-and-curl trial passes the recorded mechanical/finite-state/contact
  overlap/stability screens, with cylinder and Z-bar pin gaps below 0.05 mm.
  Maximum mass residual and boundary loss are both 0 kg. All five failed
  single-run checks concern entry, positive payload or its support/retention.
  Mass closure with an empty bucket is not successful scooping.
- Independent zero-clearance centroid replay also sees no inside grains.
  Therefore half-plate measurement clearance alone does not explain this zero.

Legacy still curls toward its carry target during the subsequent lift phase;
the defect concerns skipping the intended pre-lift curl phase, not absence of
all curl motion. The opt-in variant changes curl/lift coordination and braking.

The one-tick curl defect is removed in the opt-in controller, but acquisition is
still unresolved. A short reaction transient now stops advance before entry,
and subsequent curl occurs without material entering the mouth. This observation
does not uniquely separate coarse-contact response, local material coverage,
cutting-edge geometry or the intended force-stop trajectory. Next isolate these
at the cutting edge before changing force thresholds, filtering or contact laws.
Do not increase the threshold merely to obtain more payload. Keep this variant
experimental; all previous shallow/spillage/large-CPU failures remain reported.

The completed candidate repeat has identical sample times and **zero maximum
recorded generalized-position difference**, including all grain poses. Prepared
and settled hashes match the old baseline and both fresh-feedback trials. The
comparison configuration/initial-state/repeat checks all pass; acquisition
acceptance still fails. An earlier interrupted repeat produced no final report
and is excluded from these checks.

Validation: 148 core tests and 11 USD integration tests pass; all 85 active-v1
asset manifest hashes remain unchanged. No vehicle asset, material calibration,
force limit, or training path is changed by this investigation. The existing
v0/v1 and reservoir-study outputs remain preserved. Local replay and comparison
evidence: `outputs/wa320/m2/curl_transition/stop_and_curl/scripted_scoop_replay.mp4`
and `outputs/wa320/m2/curl_transition/controller_comparison.{json,png}`.
