# Replay state and cylinder appearance

- Render a recorded full maneuver with `MUJOCO_GL=egl python scripts/render_wa320_replay.py --input outputs/wa320/v_cycle/nominal --speedup 4`.
- The video reconstructs saved root and joint configurations. It does not advance dynamics. Use `--config` to select the matching vehicle configuration.
- The right panel reports cabin centre, front-frame hinge ("head") and bucket pivot. Cabin is rigidly attached to the rear frame; it has no independent suspension in this model.
- W is the fixed simulation world: Z up, X along the initial forward direction, Y initially left. Positions and linear/angular velocities use W axes. RPY uses `R = Rz(yaw) Ry(pitch) Rx(roll)`; displayed angles are degrees, angular velocities rad/s.
- Velocities use adjacent recorded configurations and simulation timestamps, independent of playback speed. Rotational velocity uses the rotation-matrix logarithm, avoiding a false spike at the yaw wrap. These are reconstructed derivatives, not independently recorded sensor measurements.
- Articulation, boom, bellcrank and passive bucket angles are relative joint displacements from the configured URDF nominal offsets. They are not absolute world angles or cylinder strokes.
- A matching `.telemetry.jsonl` file includes timestamps, world poses, velocities, joint states and visual pin endpoints. Its header records units and provenance. Raw output stays ignored by Git.

## Geometry and unavailable states

- The grey rectangular member in the original contact model is the rigid Z-bar connecting rod, not a piston. Replay displays a round rod with pivot eyes instead.
- Replay adds two lift cylinders and one tilt cylinder with yellow barrels, chrome rods, collars and pivot pins. They track attachment points on the recorded front frame, boom and bellcrank. They do not add rigid bodies, collisions, constraints or forces.
- Lift mounts are configurable visual estimates: front-frame `[0.78, ±0.68, -0.05]` m to boom-local `[1.20, ±0.57, -0.10]` m. The earlier negative-X base lay behind the central hinge and swept into the rear box; the correction keeps the estimated pins on the front assembly. Tilt uses the existing configured front-frame base and upper bellcrank pin. These mounts are not manufacturer measurements or a verified mechanical ratio.
- Barrel length is fixed at 60% of the first recorded pin distance. Radius is 95 mm for lift and 110 mm for tilt, both visual estimates. This is appearance, not a cylinder specification.
- L is current pin-to-pin distance. dL is change since the first saved sample, not extension from a measured retracted stop. Length rate is derived from those distances. Both endpoints are shown in W; their vector defines the cylinder axis.
- Absolute stroke, retracted/extended stops, pressure, flow and hydraulic force are unknown. The physical model still uses equivalent rotary drives. The cylinder rendering and right panel currently belong to recorded replay, not a live Isaac viewer or hydraulic simulation.
- Improved appearance does not establish correct cylinder placement, collision-free packaging or real-machine fidelity. See [validation basis](VALIDATION_BASIS.md).

- Floor appearance defaults to procedural off-road ground; select `--surface asphalt` for asphalt. Physical ground properties are unchanged; see [surface and rigid-pile checks](RIGID_PILE_CYCLE.md).
