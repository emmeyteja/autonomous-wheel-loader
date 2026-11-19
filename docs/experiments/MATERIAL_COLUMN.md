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

## Recorded diagnostic results

- Gravel: all numerical screens passed for nominal, repeat seed, half timestep and finer elements. This establishes a working diagnostic, not physical calibration or a completed excavation benchmark.
- Nominal: 699 elements, 1 ms step, 13.744 kg represented mass, 0.153 m final height and 0.336 m p95 runout radius. Sector fits span 29.9–34.9°, with only 4–5 bins per fit. These are apparatus-specific fits, not an independently validated repose angle.
- Halving the timestep gives 0.153 m height and 0.331 m p95 runout. At the same 0.5 ms step, 1456 finer elements give 0.151 m height and 0.299 m p95 runout, about 9.6% lower. The finite lattice also changes represented mass to 14.616 kg, about 6.3% higher; this is an exploratory resolution sensitivity test with a packing/mass confound, not a controlled convergence result. Match preparation and bulk mass before a convergence claim.
- Dry-sand candidate: rejected by the 1 m runout screening window despite settling; p95 runout is 0.713 m and all sector slopes are unavailable. The option remains experimental. Do not widen the window or immobilize grains to disguise excessive spreading; calibrate against a matched sand experiment.
- Next physical validation requires measured column preparation, mass/density, surface friction and collapse profiles for the selected material. Then add matched tool tests before claiming bucket-force or pile-relaxation accuracy.
