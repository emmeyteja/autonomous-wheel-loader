# Isolated material contact diagnostic

- Native CPU MuJoCo diagnostic in the simulation integration, independent of the Newton vehicle scene. It adds no replacement vehicle simulator or production granular terrain backend. Newton/Isaac coupling and batched training remain unavailable until separately validated.
- Gravel is the default trial; `--material dry_sand` retains a separate trial. Both remain **not validated**. Production material YAMLs retain unknown calibration values and do not unlock granular loading or PPO.
- `configs/experiments/material_column.yaml` explicitly separates trial contact coefficients from published reference properties. MuJoCo rolling/torsional coefficients have length units; they cannot be copied numerically from a different DEM rolling law. See [MuJoCo contact semantics](https://mujoco.readthedocs.io/en/stable/computation/#contact).
- Gravity releases a slightly polydisperse, nonoverlapping column with persistent element IDs. No walls, frozen elements, imposed final slope, cohesion or extra joint damping maintain its shape. Column radius is 0.12 m, nominal height 0.25 m, nominal element radius 12.5 mm. These are diagnostic geometry/resolution choices, not a reproduction of natural gravel/sand gradation or Aoshima's field pile.
- Six-dimensional elliptic contacts, CG with 40 iterations and a 2 ms numerical contact reference are explicit. Default timestep is 1 ms. A 256 MiB arena and bounded geometry generation prevent unbounded allocation; every suite case runs in a separate process.

## Reproduce

    pip install -e '.[material-audit]'
    python scripts/check_material_column.py --material gravel --suite --output outputs/material_column/gravel
    MUJOCO_GL=egl python scripts/render_material_column.py --input outputs/material_column/gravel/nominal
    python scripts/check_material_column.py --material dry_sand --output outputs/material_column/dry_sand

- Cases: nominal seed 42, repeat seed 43, half timestep, and finer 10 mm elements at the half timestep. Compare nominal/half-dt for temporal sensitivity, and half-dt/finer for spatial sensitivity at matched timestep. Agreement is measured evidence, not presumed convergence.
- Numerical checks cover finite states, solver warnings, represented element mass/IDs, runout below the 1 m diagnostic window, penetration below 15% of the minimum element radius, and two trailing seconds with p95 linear speed below 5 mm/s, rim speed below 10 mm/s and p90-radius drift below 5 mm. These are project screening limits, not laboratory tolerances or physical material validation.
- Four sector surface fits report angle, fit quality and sample bins. Sparse/flat profiles return unavailable. A shape fit is not a calibration claim; the different release apparatus has no matched physical reference recording here.
- Effective XML, positions, linear/angular velocities, masses, energy, minimum separation and validation JSON stay under ignored outputs. Renderer uses recorded positions only, labels candidate/calibration status, and rejects a run with failed numerical screens.
- A coarse sphere candidate can settle while spreading too far or producing shallow/poorly resolved slopes. Do not treat a stable picture as proof of realistic sand or gravel. Density, surface friction, shear behavior, particle-shape sensitivity, matched collapse data and excavation remain separate requirements.
- Published full-loader gravel reference, other materials and cross-vehicle comparison rules: [benchmark applicability](../benchmarks/MATERIALS.md).

## Controlled preparation and comparisons

- Default target mass is 13 kg at the selected solid density. Generate the seeded lattice, shrink all radii by one common factor to match solid volume, then expand centre separations horizontally and vertically to match the 0.12 m outer radius and 0.25 m total height. Expansion and radius reduction preserve nonoverlap and every particle ID. Targets requiring inflation are rejected. No solver mass/density override conceals a geometry mismatch.
- Initial velocity is zero. This is a reproducible ordered diagnostic fabric, not settled laboratory preparation. Record actual element radii after normalization; the configured radius is a generation scale, not a measured grain distribution. Bulk mass divided by the prescribed cylindrical envelope is about 1149 kg/m³, not Aoshima's measured 1727 kg/m³.
- `suite.json` retains each numerical screen. `comparisons.json` reports seed, timestep and element-size sensitivity only if numerical checks, mass, measured envelope, preparation, material/contact settings and the relevant timestep controls match. Failed/missing controls suppress comparison metrics. Neither file can establish physical validation or convergence.
- Fixed length-valued rolling/torsional coefficients change relative to particle size on refinement. The small ordered column, particle shape, fabric and boundary-to-grain ratio remain limitations even after mass is controlled. Do not call this a calibrated mesh-convergence study.
- To reproduce the original unequal-mass preparation, use a separate config with `target_mass_kg: null`. Such runs do not pass the controlled comparison gate.
- [Selected physical reference and independently checked profile-measurement method](../benchmarks/GRAVEL_HEAP_PROTOCOL.md). The slow-lift apparatus must be reproduced before any matched material-accuracy claim.

## Controlled diagnostic results

- Nominal, repeat seed, half timestep and finer-element trials all pass numerical checks, mass/envelope checks and comparison eligibility. All retain physical-validation status false.
- Nominal: 699 elements, 13.000 kg, 0.147 m final height and 0.336 m p95 runout radius. Actual element radii are 11.66–12.27 mm after mass normalization.
- Half timestep: 0.148 m height and 0.341 m p95 runout, a 1.67% runout increase. At the same 0.5 ms timestep, 1456 finer elements of radius 9.14–9.62 mm give 0.137 m height and 0.302 m p95 runout: 11.51% lower runout and 6.89% lower height despite matching mass and initial envelope. Seed change gives 3.53% lower runout.
- Resolution dependence remains substantial. Two element sizes and one coarse-element timestep pair do not establish convergence, especially with ordered preparation and fixed length-valued friction coefficients. Finer-element temporal sensitivity is not established by this suite.
- The checked video is a 1× replay of the nominal 12-second run. This is a numerical material diagnostic, not calibrated gravel, slow-lift reproduction or bucket loading. The physical reference and its measurement-method check are documented separately.

## Earlier unequal-mass diagnostic results

- Gravel: all numerical screens passed for nominal, repeat seed, half timestep and finer elements. This establishes a working diagnostic, not physical calibration or a completed excavation benchmark.
- Nominal: 699 elements, 1 ms step, 13.744 kg represented mass, 0.153 m final height and 0.336 m p95 runout radius. Sector fits span 29.9–34.9°, with only 4–5 bins per fit. These are apparatus-specific fits, not an independently validated repose angle.
- Halving the timestep gives 0.153 m height and 0.331 m p95 runout. At the same 0.5 ms step, 1456 finer elements give 0.151 m height and 0.299 m p95 runout, about 9.6% lower. The finite lattice also changes represented mass to 14.616 kg, about 6.3% higher; this is an exploratory resolution sensitivity test with a packing/mass confound, not a controlled convergence result. Match preparation and bulk mass before a convergence claim.
- Dry-sand candidate: rejected by the 1 m runout screening window despite settling; p95 runout is 0.713 m and all sector slopes are unavailable. The option remains experimental. Do not widen the window or immobilize grains to disguise excessive spreading; calibrate against a matched sand experiment.
- Next physical validation requires measured column preparation, mass/density, surface friction and collapse profiles for the selected material. Then add matched tool tests before claiming bucket-force or pile-relaxation accuracy.
