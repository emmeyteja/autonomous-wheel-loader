# Instrumented blade diagnostic

- Isolated native CPU MuJoCo diagnostic; no vehicle simulator or production granular backend is replaced. Requires the existing `material-audit` extra and ffmpeg for replay.
- Runs all nine widths/depths from the [Obermayr reference](../benchmarks/OBERMAYR_BLADE.md), but remains **unmatched and uncalibrated**. Fill, preparation, blade thickness/mass, contact coefficients and fitting interval are trial assumptions. Current grains are substantially coarser than the paper; shallow cuts and narrow blades are poorly resolved.
- `configs/experiments/blade_force.yaml` selects gravel by default. Dry sand remains a separate experimental option, not a validated material.

## Reproduce

    pip install -e '.[material-audit]'
    python scripts/check_blade_force.py --matrix --jobs 3 --output outputs/blade_force/matrix
    python scripts/check_blade_force.py --empty --output outputs/blade_force/empty
    python scripts/check_blade_force.py --half-dt --output outputs/blade_force/half_dt
    python scripts/check_blade_force.py --half-dt --finer --output outputs/blade_force/finer
    python scripts/check_blade_force.py --material dry_sand --output outputs/blade_force/dry_sand
    MUJOCO_GL=egl python scripts/render_blade_force.py \
      --input outputs/blade_force/matrix/w100_d40 \
      --output outputs/blade_force/matrix/w100_d40/blade_cut.mp4

- `--jobs` bounds independent matrix processes to 1–3, each with a 256 MiB solver arena; it is not a batched training path. `--width` and `--depth` use metres. A fresh scene is constructed per process/case. The native integration test separately resets and repeats the same loaded scene using the same model/data.
- The 10 kg initial bed occupies X=0.04–0.29 m, Y=±0.14 m, Z=0–0.12 m. Seeded, ordered nonoverlapping preparation is mass-normalized by reducing grain radii and expanding centre separations. It is not measured laboratory preparation. Particle IDs are never deleted and rotations remain free.
- Phases: 3 s settling; 1 s smooth vertical blade insertion; 2 s hold; 80 mm forward cutting at 10 mm/s; 2 s relaxation. Cutting depth uses the grain-top H95 before insertion, a rough surface proxy. The bottom must remain at least 15 mm above the floor. Insertion changes the bed and is not a reproduction of the paper's unspecified initial blade placement.
- A regular dynamic blade follows a mocap target through a weld constraint, preserving blade velocity in contacts. A directly moved mocap collider would omit that velocity; see [MuJoCo modeling](https://mujoco.readthedocs.io/en/stable/modeling.html#mocap-bodies). The weld uses a 1 m torque scale to resist fixture tilt, a 2 ms time constant and a provisional 10 kg body mass. These numerical drive settings are not physical blade dimensions or hydraulic parameters. A smaller 0.1 m torque scale failed the unchanged 0.01 rad orientation gate in deep cuts.
- Grain, wall/floor and blade friction are separate. Boundary priority overrides grain friction when forming contacts. Collision masks admit blade/grain contacts and exclude blade/wall contacts; travel and depth stay inside the bin. Walls retain grains; they are not frozen or slope-constrained.

## Measurement and checks

- Frame: X travel, Y blade width, Z up. `force.csv` records solver-rate world reaction force [N] and moment [Nm] about the blade origin, X/Z position [m], X velocity [m/s], time [s] and phase; full XYZ pose/velocity are retained in the recording. Phase IDs 0–4 follow the order above.
- Contact wrenches are transformed from the solver contact frame with the appropriate geom-side sign and moment arm. Only material contact forces are reported, excluding gravity and weld reactions. A 2 kg known-load test verifies +19.62 N vertical support and equal/opposite ground reaction.
- Split stepping pairs pre-integration positions/velocities with the contact solve at the same timestamp. `motion.npz` retains 25 Hz scene states plus full-rate forces, poses, velocities and targets. Replay uses these recorded states, not new physics.
- Draft is H=-Fx for forward cutting. Fit H=H0+slope*x over the explicit 20–70 mm travel window, excluding insertion and relaxation. This interval is a diagnostic choice; the source interval/raw trace is unavailable. Report residual RMS. An extrapolated H0 can be negative for a nonlinear entry curve; do not clamp it or tune the window to hide it. It is not the mean force or a measured breakout limit.
- `cut_work_J` integrates H dx over forward cutting only. It is horizontal tool work, excluding insertion, rotations, hydraulics and vehicle energy. No physical comparison is computed without a matched reference.
- Numerical screens: finite state/wrenches; no solver warnings; overlap below 15% of the smallest radius; blade position error below 2 mm and angle error below 0.01 rad; p99 cut-speed error below 1 mm/s; represented mass conserved; all grains inside the bin; p95 speed below 5 mm/s in the trailing settling/hold windows; positive opposing draft/work in loaded cases or zero material force in the empty case. These are project diagnostic gates, not laboratory accuracy tolerances.
- Spatial sensitivity holds initial mass/envelope and half timestep fixed. Different ordered fabric and length-valued rolling coefficients relative to grain size still affect results. Timestep/resolution agreement cannot establish physical calibration.
- The production granular backend, bucket loading and PPO gates remain unavailable. Independent shear/tool-friction calibration and matched force records are required before interpreting this as soil-force validation.

## Numerical interpretation

- All nine gravel width/depth combinations pass the stated numerical screens at 1 ms. Maximum blade translation error across the matrix is 0.853 mm and maximum tilt is 0.000435 rad. The default dry-sand candidate also passes these screens; it remains uncalibrated.
- With MuJoCo 3.12.0, the 100/40 mm gravel case uses 528 grains and 10 kg. At 1 ms, horizontal cutting work is 0.743 J, H0 is -2.29 N and fit RMS is 2.50 N. At 0.5 ms with the identical initial grain-state hash, work is 0.795 J (+6.99%), H0 is 8.39 N and fit RMS is 3.16 N. The intercept is not timestep-converged; reporting a relative error against the near-zero nominal intercept would be misleading.
- The empty control produces zero material reaction and cutting work. The known-load and repeated-reset integration controls pass. Passing numerical screens establishes bookkeeping and bounded execution only, not material or excavation accuracy.
- At the same 0.5 ms timestep, refining from 528 to 1,187 grains preserves the 10 kg initial mass/envelope and passes the numerical screens. Work changes from 0.795 to 0.536 J (-32.5%), and H0 from 8.39 to -5.38 N. Particle-size/fabric sensitivity remains substantial; this is not a spatially converged result. Resolve timestep, preparation and contact-law sensitivity before matching measured blade forces or coupling to the loader.
