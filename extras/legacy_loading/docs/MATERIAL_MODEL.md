# Material checks

- The target is dry sand. The current model uses coarse rigid spheres and MuJoCo Warp contacts through Isaac Lab/Newton; it is not calibrated soil.
- Keep a stockpile preview, a free-column collapse check, and a moving-bucket probe separate. A stationary flat layer can pass the motion screen while failing the shape check.
- `configs/sand_column.toml` releases a 0.6 m diameter, 0.65 m high column of 600 spheres, each 0.06 m diameter. There are no retaining walls, cohesion, frozen particles, or added velocity damping.
- Run `.venv/bin/python scripts/check_sand.py` from the repository root. It sweeps rolling resistance at seeds 42 and 43, with one fresh simulator process per case; summaries and effective configurations stay under `outputs/sand_check/`.
- The reference shape screen requires all four fits to have R² >= 0.8 and angles within 34° ± 5°. This is a provisional comparison; `calibration_status` remains `not_validated`.
- Motion screen: over at least two trailing seconds, 95th-percentile speed <= 0.02 m/s, <= 1% speed samples above 0.05 m/s, and 90th-percentile footprint-radius change <= 0.02 m. These are project screening thresholds, not laboratory standards.
- Side-slope diagnostics fit sphere-top envelopes in diameter-wide bins along four central faces, using 20–80% of peak height and excluding the ground layer. Sparse profiles return unavailable; report fit quality instead of inventing an angle.
- [Dastgerdi and Malinowska (2024), sections 3.1–3.3](https://www.mdpi.com/1996-1944/17/19/4833) report dry quartz sand with 0.45 mm median grains, about 2600 kg/m³ solid density, and a 34° mean funnel-test repose angle. Their calibration uses non-spherical, much smaller particles and LS-DYNA; its contact coefficients cannot be copied into this backend as validation.
- The 2600 kg/m³ value in the column and loader-probe presets is solid-particle density; bulk density must be measured from the resulting packing. Existing stockpile previews retain their earlier uncalibrated density assumption.
- MuJoCo represents a single sliding coefficient rather than separate static/dynamic values. Inspect the exported `friction` values; the current Newton translation exports `dynamic_friction` in the sliding slot. Six-dimensional contacts are necessary for rolling resistance.
- Matching a 34° reference alone would still require bulk-density, grain-resolution, time-step, repeat-seed, and excavation-response checks. Column collapse is not the same apparatus as a funnel test.

## Loader interaction probe

```bash
./scripts/demo_pile.sh --config configs/loader_probe.toml --loader-probe --duration-s 30 --output outputs/loader_probe
```

- The [authored approximation](../src/wheel_loader_rl/assets/README.md) settles the pile, approaches, curls/lifts, withdraws, holds, then tips forward using an open-loop joint-target sequence.
- Grains move only under gravity and contact; none are attached to the bucket or teleported into it.
- Checkpoints use whole-sphere containment at 16 s (acquired), the same IDs at 22 s (retained), and release at 30 s. Boundary grains are excluded with a 0.005 m contact tolerance. Incomplete runs cannot pass the transfer screen.
- A demonstration ground pad spans x=[-2,1], y=[-2,2] m, with deposited sphere tops <= 0.6 m. It is not a dump bed or autonomous zone planner.
- Final bucket, pad, source-region, and other/spillage categories partition all particle IDs. Zero partition error proves fixed-particle bookkeeping only; it does not validate continuum material conservation or a payload sensor.
- Verify a complete recording with `.venv/bin/python scripts/check_loader_probe.py outputs/loader_probe`; the command fails for missing acquisition, retention, or pad deposition.
- Successful acquisition/retention/deposition is a material-backend feasibility check. It does not establish traction-limited loading, learned control, accurate bucket forces, batched task reset, or M0 completion.
