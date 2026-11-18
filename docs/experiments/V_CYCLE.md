# V-cycle motion rehearsal

- Unladen rigid-ground evaluation using the existing Newton integration and WA320-like approximation.
- No sand pile, material pickup, retained payload or material transfer is simulated. This does not complete the granular-loading milestone.
- Every case runs the entire sequence: settle → forward-right → lower to attack → short forward push → curl/lift → reverse to center with the bucket raised → forward-left → raise → dump motion → rack back → reverse home → lower → stop.
- Cases: nominal (12 m forward, ±6 m lateral, 0.6 m/s maximum), slow (same geometry, 0.45 m/s), wide (15 m forward, ±7.5 m lateral, 0.6 m/s). Waypoints describe the rear-frame hinge reference, not the bucket lip.
- control/v_cycle.py owns the feedback controller and phase state. It samples a short articulated-kinematic prediction, including steering-rate reaction at the hinge, to choose steering. It is a deterministic evaluation controller, not a trained policy or learned world model.
- Commands ramp speed, steering, boom and bellcrank within bounds. Bucket/rod positions remain passive physical outputs. Physics supplies measured pose/joint feedback; no pose animation or teleporting is used during the maneuver.
- Named scenarios are single-environment evaluation runs in separate processes. They are not a batched training implementation.

## Reproduce

In the configured Newton environment:

    python scripts/check_wa320_v_cycle.py --case all --output outputs/wa320/v_cycle
    MUJOCO_GL=egl python scripts/render_wa320_replay.py --input outputs/wa320/v_cycle/nominal --speedup 2

- --case nominal, --case slow, or --case wide selects one complete maneuver.
- Each case writes motion, commands, phase labels, forces, the effective solver model and validation results under its output directory. suite.json reports all three cases; the suite exits nonzero if any case fails.
- Video is a labeled replay of measured poses, with a recorded-route inset. Playback speed is explicit. It does not run new physics or depict acquired sand.

## Acceptance

- Complete every phase in order. Verify measured travel direction during both forward legs, the loading push and both reverse legs.
- Reach the right/left endpoints and return through the V center; final position must be within 0.35 m of the origin.
- Verify boom/bucket motion during curl/lift, boom above 0.3 rad while reversing from the loading location, bucket motion during dumping, zero final speed command and measured final planar speed below 0.02 m/s.
- Travel legs must stay within a 3.5 m centerline corridor and below 1.6 times their direct length. The corridor allows curved steering transitions; it rejects large detours despite correct endpoints.
- Retain diagnostic joint limits, finite states, passive closure, ground/self penetration, force support and floating reset checks. These checks retain their existing thresholds.
- Unit tests run the full controller sequence for every case and reject missing travel, skipped dump, missing bucket motion, wrong reverse direction, a lowered bucket during reverse, path detours and an incomplete return.
- Estimated dynamics, simplified rigid tires and numerical contact compliance remain uncalibrated. No loaded stability, excavation force, dumping efficiency or full-workspace collision guarantee is implied.

## Validation record

- Nominal: full sequence in 175.0 s; final origin error 0.245 m; simultaneous four-wheel support 97.80%; maximum passive-loop gap 0.128 mm.
- Slow: full sequence in 217.9 s; final origin error 0.129 m; simultaneous four-wheel support 98.69%; maximum passive-loop gap 0.127 mm.
- Wide: full sequence in 196.2 s; final origin error 0.181 m; simultaneous four-wheel support 98.00%; maximum passive-loop gap 0.126 mm.
- All three pass the same complete-maneuver, route, signed lift/curl/dump, gear-direction, contact, limits, closure and floating-reset gates. Final measured planar speeds are below 0.001 m/s.
- Saved trajectories were rechecked against the common acceptance function. Logs, motion and videos stay under ignored outputs.
- 61 core tests and 4 asset/integration tests pass. The simulator cases are optional and require the configured Newton environment.
- Tracking: [complete V-cycle maneuver](https://github.com/emmeyteja/autonomous-wheel-loader/issues/6).
