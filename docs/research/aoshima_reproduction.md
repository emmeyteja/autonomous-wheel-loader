# Aoshima reproduction audit

Scope: source verification and reproduction requirements, not a claim that this
repository reproduces the papers. The two user-supplied published PDFs were read
directly, including rendered Table 3 and Figure 8. Page references below use printed
journal pages; PDF page numbers are given where useful. No PDFs are redistributed.

## Sources

- **GAP:** Aoshima and Servin, *Examining the simulation-to-reality gap of a wheel
  loader digging in deformable terrain*, [DOI](https://doi.org/10.1007/s11044-024-10005-5).
  Published online 19 July 2024; supplied PDF labels volume 64, pp. 121–148 (2024).
  Publisher indexing may associate the volume with 2025; identify by DOI.
- **WM:** Aoshima, Fälldin, Wadbro and Servin, *World Modeling for Autonomous Wheel
  Loaders*, Automation 5 (2024), pp. 259–281,
  [DOI](https://doi.org/10.3390/automation5030016).

## Verified physical and numerical parameters

| Quantity | Verified value / qualification | Source |
|---|---|---|
| Field vehicle | WA320-7; 15,175 kg; 3.03 m wheelbase; 2.05 m track; 1.39 m wheels; 3 m³, 2.685 m-wide bucket | GAP §4.1, p.126 / PDF 6 |
| Field material | Crushed/sifted sedimentary gravel, roughly 30–40 mm, with fines and some moisture; physical D10/D50/D90 not supplied | GAP §4.2, p.126 |
| Field bulk density | 1727 kg/m³, measured | GAP §4.2, p.126 |
| Field pile boundaries | Flat rigid floor; vertical side and rear walls | GAP §4.2, Fig.1, pp.126–127 |
| Reconstructed full-DEM pile | Emitted into a 6 m-wide container; front surface matched to field scans | GAP §5.3, p.131 |
| Field pile height/depth/volume | No complete dimensioned 3D pile specification in the reviewed sections/figures; scanned surfaces needed, not a guessed trapezoid | GAP §§4.2–4.3, 5.3–5.4 |
| DEM solid density | 2590 kg/m³, assigned simulation value; not a reported independent field measurement | GAP §5.3, p.131 |
| DEM contacts | Spheres; friction 0.3; AGX rolling coefficient 0.02; restitution 0; Hertz-mapped nonsmooth contact | GAP §§5.1, 5.3, p.131 |
| DEM diameter distribution | Uniform within D ± 0.1D; **numerical**, not physical grading | GAP §5.3, p.131 |
| Bulk multiscale settings | Density 1727 kg/m³; internal friction 32°; dilatancy 8°; Young modulus 4.6 MPa | GAP §5.4, p.133 |
| G-model bucket interface | Friction 0.2 and aggregate stiffness multiplier 0.01 calibrated to dig forces; edge radii 2.5–10 mm, equivalent tooth length 10 mm | GAP §5.4, p.133 |
| Dry-gravel WM settings | Density 1727 kg/m³; friction angle 32°; cohesion 0 kPa; dilatancy 8° | WM §5.1, p.266 / PDF 8 |
| WM numerical grid/time | 0.1 × 0.1 m terrain cells; 0.01 s timestep; AGX multiscale terrain | WM §5.1, p.266 |
| WM seed piles | Two each triangular, conical, wedged; slopes 20° and 30°, Perlin perturbations | WM §6.2, p.267; Fig.8, p.268 |
| WM dimension labels | Fig.8: triangular span 30 m and transverse extent 5 m; conical radial label 5 m; wedged-seed plan-view annotation 45°. These do not specify all 3D heights/volumes | WM Fig.8, p.268 / PDF 10 |
| WM performance observation window | 3.6 × 3.6 m, 36 × 36 cells | WM §6.3, p.268 |
| WM next-pile observation window | 5.2 × 5.2 m, 52 × 52 cells; displacement 1 m toward approach side | WM §6.3, p.268 |

The heightmap windows are **local observations**, not the physical pile footprint.
The field test includes some moisture; the WM paper's idealized dry gravel is not
proof that the field sample was perfectly dry. Physical PSD percentiles, measured
particle friction, temperature and moisture fraction remain unknown.

### Resolution table, directly checked against GAP Table 3 (p.129 / PDF 9)

| Configuration | Characteristic diameter/grid [mm] | Timestep [ms] | Particle count | Solver iterations |
|---|---:|---:|---:|---:|
| D50 | 50 | 2 | 450,000 | 200 |
| D100 | 100 | 5 | 55,000 | 100 |
| D200 | 200 | 10 | 6,100 | 500 |
| D400 | 400 | 20 | 800 | 15 |
| G50 | 50 | 5 | ≲28,000 | 50 |
| G100 | 100 | 10 | ≲3,500 | 25 |
| G200 | 200 | 20 | ≲480 | 15 |
| G400 | 400 | 50 | ≲55 | 10 |

D means full-pile nonsmooth DEM. G means multiscale grid/active-zone resolution;
G counts are approximate active-particle bounds, not fixed whole-pile counts.
D50 is a fidelity label, **not** the physical PSD median D50. Preserve the reported
D200 iteration count of 500; do not silently replace it by an interpolated value.

## What the G-model actually does

GAP §5.4 and Fig.4 (p.132), continuing p.133; WM §5.1 (p.266):

1. Resting soil occupies a voxel grid with mass occupancy and compaction, with a
   heightmap surface reconstructed from scans.
2. A bucket-dependent shear wedge predicts the active region.
3. Local DEM represents moving material; particles may change size/number.
4. An aggregate rigid body carries active mass, shape, inertia and momentum,
   coupling soil reaction back into the vehicle's multibody solve.
5. Local particle co-simulation sees the moving bucket kinematically. Its reaction
   is mediated through the aggregate, with distinct soil/soil and soil/bucket laws.
6. A penetration constraint accounts for cutting-edge resistance. Resting particles
   can merge into continuous soil while conserving mass.

A heightmap with decorative particles, or a kinematic bucket replay through a
particle pile, does not reproduce this coupling. Implementing this from scratch
would require conservative mass/momentum exchange and independent force validation;
AGX's implementation is not provided by these papers as portable project code.

## Validation, data and limits

- GAP Table 1 (§4.3, p.127): 100 Hz field measurements; cylinder-pressure-derived
  forces, wheel-derived speed, linkage measurements, rear-wall laser distance and
  pre-loading surface scans. Some forces/extensions are published in normalized
  arbitrary units; raw SI signals/calibration constants must be obtained. The data
  availability statement (p.146 / PDF 26) offers normalized time series on request;
  it does not establish that SI field data are available to this repository.
- GAP Table 2 (p.128): FB35 = 3.46 t / 209 kJ; HD27 = 2.70 t / 127 kJ;
  RD21 = 2.10 t / 112 kJ. RD21 starts with tip 0.25 m above ground.
- GAP §§6.1–6.3 (pp.134–135): identify target-speed commands/calibration using
  G200, then reuse commands across fidelities. Use normalized scalar MAE and
  normalized DTW for bucket-tip paths, with stated phase exclusions.
- GAP §§6.5–6.6 (p.136), Tables 4–5 and Fig.9 (p.138): mean absolute loaded-mass
  errors about 15% (D), 12% (G); work 17%/21%; combined sim-to-real error about
  10%, standard deviation 3%. This is not a 10% bound on every output/material.
- Mechanical work is the integral of drive and cylinder power; engine,
  transmission and hydraulic losses are excluded (§4.3, p.127).
- GAP §8 (p.141): homogeneous gravel only; simplified power-sharing/hydraulics.
  Resolution convergence and successful controller transfer do not prove fidelity
  for coal, mixtures or wet soil.
- WM §6.2 (p.267): 10,718 samples, repeated sequential loading from six seeds.
  These are the authors' simulations, not a dataset currently installed here.
- WM Table 1 (p.269): high-dimensional models' mass/time/work MRE spans
  3.61–6.98% across targets/activations. Table 2 (p.271): next-pile volumetric
  MAE 0.75 m³, MRE 3.04% with post-processing. Targets are **simulator outputs**.
- WM §§4.4–4.5, Figs.3–4 (pp.264–265): CNN/MLP performance predictor and
  VAE-latent transition pile predictor; material conditioning was not demonstrated.
- WM Table 3/Figs.13–14 (pp.273–274): 40-cycle predictions accumulate surface
  error; scalar and next-pile accuracy need separate evaluation.
- WM §4.7 (p.265), §8.1 (p.275): homogeneous gravel, no post-breakout ground
  spillage/full V-cycle; smoothed pile predictions may violate repose during long
  rollouts. Cross-material transfer is a research hypothesis, not a paper result.

## Repository audit and reuse

- WA320 reconstructed v1: `vehicle/`, `configs/vehicle/wa320.yaml`, original assets;
  measured/reference/estimated provenance remains mixed. No CAD-complete claim.
- M2: `control/scripted_scoop.py`, `environment/simulation/scoop_probe.py`,
  `terrain/bed.py`, `evaluation/material_accounting.py`; native force-driven
  cylinders, free coarse spheres, exclusive ID ledger and mouth events.
- Current benchmark: 83 numerical spheres, ~0.228–0.240 m diameters. Six retained
  elements sum to 102.309 kg. This is neither physical gravel grading nor a
  calibrated sand/coal model. Centroid membership counts whole particles.
- `terrain/rigid_pile.py` is an immovable approach obstacle. `granular_dem`,
  `multiscale_dem`, `heightmap` registry entries deliberately remain unavailable.
- Existing column/blade diagnostics and pile/force/accounting evaluators are useful
  independent calibration fixtures. Reuse them before a new terrain framework.
- [Backend benchmark](../experiments/PHYSICS_BACKEND_BENCHMARK.md): native repeat
  exact; direct MJWarp diverged and warned about solver iterations; PhysX ran but
  its contact/material/collision mapping was non-equivalent. No backend selected
  for accurate granular RL. Newton MPM coupling was not tested.
- Existing material YAML files are null placeholders with ambiguous legacy names
  such as `density`. Preserve them and M2 experiment settings; add versioned,
  explicit profiles without silently activating them in a simulator.
- `training/cli.py` offers foundation checks and explicitly rejects PPO. No active
  world-model implementation, dataset contract, fitted weights or transfer results
  exist. Reuse module boundaries and sensor availability conventions; propose
  cycle-transition records without introducing a trainer.

Reproduction still needs original scanned pile geometry, field target-speed and
force histories with scaling constants, material calibration apparatus/data and
vehicle inertia/hydraulic details. Do not digitize photographs into exact dimensions
or compare WA320 payloads against unmatched piles/controls as an accuracy test.
