# Isolated cutting-edge entry diagnostic

## Purpose and reuse

- Investigate [the guarded scoop's zero acquisition](GEOMETRY_GATED_SCOOP.md) before changing vehicle geometry, material parameters or controller gains.
- Reuse the preserved bundle's complete bucket colliders, floor and first particle's radius, mass/inertia and contact properties. The readable export rounds some values; restore authoritative compiled geometry/material arrays and bounds before stepping. Replay uses the saved binary fixture, not its approximate readable XML. Tests verify restoration, including mesh vertices.
- Reuse `MaterialAccounting`, `AcquisitionFunnel` and the existing bucket occupancy classifier. No second terrain system, filling rule, payload assignment, particle relocation or attachment.
- Deliberately omit chassis, wheels, closed-loop linkage, cylinders and other grains. The bucket is a dynamic three-axis carriage (X, Z, pitch) driven by bounded generalized forces; this is an isolated contact experiment, **not a WA320 controller or an M2 scoop**. Direct pitch targets belong only to this fixture; the full loader continues to use physical cylinder commands.

## Configuration

- Native MuJoCo Newton solver; original timestep 1/1920 s, 240 Hz observations, 16 s per case. Log native warnings every physics step; forces/normals at observation ticks. No GPU comparison or training-throughput claim in this fixture.
- One particle, or three in a transverse row spaced 2.2 radii. Copy the first reference particle: radius 0.116555293 m, mass 17.178465592 kg. This intentionally removes radius variation and overburden; it is not literal sand or a representative soil volume.
- Start centres at X=0.5 m and Z=radius+0.001 m; gravity settles them. Start cutting edge at X=0, Z=0.085 m, pitch −0.05 rad. Hold one second; advance carriage at 0.12 m/s for fourteen seconds; hold afterward.
- Three paths: no curl; curl after nominal 0.40 m entry; curl after nominal 0.95 m entry. Entry reference is the **initial** particle front X=0.5−radius, not the moving particle front. Curl adds 0.25 rad at 0.20 rad/s while holding bucket-origin Z. Every case has identical initial state and parameters for its particle count.
- Finite carriage PD gains: proportional [200000, 200000, 100000] in N/m, N/m, Nm/rad; damping [40000, 40000, 20000] in Ns/m, Ns/m, Nms/rad. Add MuJoCo bias compensation; clamp each generalized force/torque to ±300000 N/N/Nm. These are diagnostic carriage settings, **not Komatsu hydraulic ratings**. Actual effort and tracking errors are recorded.
- Particle states are set only at initialization. Subsequent motion uses gravity/contact forces. There is no kinematic boundary during accepted runs. An initial mocap prototype is excluded from the accepted results.

## Results: six cases, three repetitions each

- Every recorded generalized position, contact row, accounting value and carriage effort repeats exactly across three runs per case.
- **Zero mouth entries and zero acquired/retained mass in all six cases**, including later curl. This is a valid diagnostic result, not a passing scoop.
- First bucket/particle contact is the original first bottom plate (`box_12_4`). Normal elevation is **0.7842°** for one particle and **0.7831°** for the row: almost horizontal, pointing forward into the particle.
- Particles remain at ground height: observed maximum centre Z is 0.117553 m, their initial radius+1 mm. They do not climb into the cavity.
- Forward displacement per particle: approximately **1.301 m straight**, **0.452–0.453 m with 0.40 m curl**, and **1.002–1.003 m with 0.95 m curl**. Delaying curl prolongs pushing in these isolated cases.
- Maximum carriage tracking errors across all cases: less than 3.02 mm in X, 1.85 mm in Z, 0.00530 rad (0.304°) in pitch. Measured bucket/floor clearance remains at least **54.16 mm**. Forces remain finite and within carriage limits; no solver warnings. Maximum particle/floor overlap is 0.167 mm (soft-contact tolerance; the diagnostic limit is 1 mm). All diagnostic gates pass.
- Initial mass is 17.178466 kg or 51.535397 kg; it remains outside the bucket. Acquisition, delivery, spillage and boundary loss are zero; maximum accounting residual is zero. Displaced particles are distinguished from material remaining in the initial source bounds. This ID accounting does not independently validate soil density or force accuracy.
- Local outputs: `evaluation.json`, contact/displacement plots and a close-up recorded replay. `states.npz` includes actual qpos, bucket poses, particle XYZ, contacts (time/index, point XYZ, normal XYZ and force XYZ on particle, distance), tracking errors, carriage efforts and complete accounting.

## Interpretation and limits

- Nearly horizontal first contact and forward displacement persist **without the loader linkage**. Later curl alone does not solve this isolated failure. This supports investigating edge/particle entry geometry before further full-vehicle control tuning; it does not establish the sole cause of failure in a confined multi-layer pile.
- The unchanged front collider is a 0.06 m thick box, not a measured cutting-edge bevel/tooth model. At the current attack height its upper edge is near the resting particle centre. Friction, edge shape and coarse particle resolution must be distinguished; material properties were not changed here.
- Peak force is sensitive to carriage stiffness, contact compliance, observation cadence and the absent pile/chassis. Do not interpret it as a real cutting resistance or cylinder force. The row has no overburden or realistic confinement; failure of a free sphere to climb is not proof that all piles fail.
- No loaded lift, reverse, retention or spillage sequence is attempted. The fixture does not validate WA320 dynamics, material calibration, a backend choice, or M2 acceptance.
- Next: a separately named, opt-in cutting-edge taper/tooth sensitivity variant against this preserved box baseline. Keep the reconstructed v1 asset unchanged. Real tip thickness, bevel length/angle and tooth spacing/shape are needed before claiming realistic geometry; hypothetical variants must remain labelled estimates.

## Reproduction

```bash
for rep in 1 2 3; do
  PYTHONPATH=src .venv/bin/python scripts/run_edge_entry.py --output outputs/wa320/edge_entry/finite_final/repeat_$rep
done
PYTHONPATH=src .venv/bin/python scripts/evaluate_edge_entry.py --root outputs/wa320/edge_entry/finite_final
PYTHONPATH=src .venv/bin/python scripts/render_edge_entry.py --root outputs/wa320/edge_entry/finite_final/repeat_1 --output outputs/wa320/edge_entry/finite_final/edge_entry_contact_replay.mp4
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q tests/integration/test_edge_entry.py
```

- Use fresh directories. Raw outputs/video remain ignored. No personal data or third-party assets are introduced.
- Five diagnostic integration tests pass, including authoritative collider/material preservation and repeatable finite-force contact. 172 core tests and seven existing mechanics integration checks also pass (184 tests total).
