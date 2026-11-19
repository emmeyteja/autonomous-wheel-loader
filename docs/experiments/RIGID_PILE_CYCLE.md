# Complete rigid-pile contact cycle

- This diagnostic extends the Newton unladen V-cycle with a fixed, non-excavatable trapezoidal prism. It tests bucket contact and subsequent curl/lift, reverse, dump motion and return, without acquiring or transferring material.
- Every named case (nominal, slow, wide) retains the complete [V-cycle sequence and acceptance criteria](V_CYCLE.md), including measured forward push above 0.25 m. A controller is not allowed to penetrate the solid pile merely to satisfy a travel target.
- Pile dimensions: 4 m length, 5 m width, 1.5 m height, 1.5 m crest length. The leading toe is 4.85 m beyond the right-side hinge waypoint along world X; its centre Y matches that waypoint. It is aligned to world axes and immovable. These are diagnostic layout estimates, not a measured stockpile.
- Pile placement leaves clearance for the lowered bucket's full contact shell, not only its lip point. The initial closer placement blocked the push prematurely; it was not accepted as a passing maneuver.
- Bucket/rod remain passive. Equivalent rotary drives, masses, source assets, tire contact geometry, friction and solver parameters are unchanged.

## Reproduce

    python scripts/check_wa320_v_cycle.py --case all --pile --output outputs/wa320/rigid_pile
    MUJOCO_GL=egl python scripts/render_wa320_replay.py --input outputs/wa320/rigid_pile/nominal --surface offroad --speedup 4

- `--case nominal`, `slow`, or `wide` runs one full maneuver. Omitting `--pile` preserves the obstacle-free motion test.
- Additional gates require bucket-to-pile geometry with nonpositive separation in the exported collision model and a runtime bucket contact-force magnitude above 1 N at a corresponding recorded sample. Contact geometry alone and force without pile contact do not pass both gates.
- The runtime sensor measures net bucket contact force. The exported-model contact sweep independently identifies pile contact; neither force nor penetration is a calibrated excavation measurement.
- Wheel support excludes pile contacts. All static contact references (ground and pile) are audited; adding a second static collider must neither fail the old single-floor assumption nor escape auditing.
- Ground/pile penetration remains limited to 10 mm, nonadjacent self-penetration to 2 mm, passive loop gap to 5 mm, with the existing joint, route, stop, force-support and floating-reset checks. Thresholds are diagnostic, not manufacturer tolerances.
- Raw results include bucket contact forces, wheel forces, poses, phase IDs, effective model and validation JSON. The suite exits nonzero if any scenario fails.
- The test uses fixed-time attachment motion and low-speed push, not a force-feedback excavation controller. It does not validate loaded stability, obstacle avoidance, the full workspace, sand flow or hydraulic forces.

## Floor appearance

- `--surface offroad` (default) shows original procedural packed-earth/aggregate texture. `--surface asphalt` selects dark aggregate texture. No external images are downloaded or redistributed.
- Surface selection affects replay appearance only. Both use the existing flat rigid plane, estimated friction and numerical contact compliance. It does not add ruts, tire sinkage, loose gravel, physical roughness or a measured asphalt/dirt material model.
- Pile replay also shows current and peak-so-far runtime bucket contact force. This is net collision force, not hydraulic-cylinder force.
- Replay includes the existing world-state panel and a `.telemetry.jsonl` sidecar recording the selected appearance. It reconstructs measured configurations without rerunning dynamics.

## Validated run
- Nominal: all gates pass; final origin error 0.214 m, simultaneous four-wheel force support 97.76%, maximum loop gap 0.125 mm, 53 bucket/pile contact samples.
- Slow: all gates pass; final origin error 0.106 m, simultaneous four-wheel force support 98.67%, maximum loop gap 0.128 mm, 20 bucket/pile contact samples.
- Wide: all gates pass; final origin error 0.178 m, simultaneous four-wheel force support 98.14%, maximum loop gap 0.126 mm, 31 bucket/pile contact samples.
- The same complete maneuver runs in all three cases. All preserve the original push, route, stop, limits, closure, contact and reset thresholds. Current tests: 64 unit and 7 integration checks pass. Raw numerical contact-force peaks are not calibrated machine or excavation-force measurements.
