# WA320 reconstructed v1 linkage and articulated structure

## Service-manual topology audit

- SEN06202-03 10-128 shows upper/lower center hinge pins and bearings. The front frame rotates relative to the rear around their common vertical axis; steering cylinders connect the two frames.
- Work-equipment disassembly 50-214–216 verifies the following relationships. Coordinates remain estimates; topology verification does not establish manufacturer pin-center dimensions.

| Manual pin | Physical connection | Model joint / constraint |
|---|---|---|
| 3 | Bucket link ↔ bucket | `zbar_closure` |
| 5 | Boom ↔ bucket | `bucket_joint` (passive) |
| 8 | Bucket-cylinder rod ↔ bell crank | `bucket_tip_closure` |
| 11 | Paired lift-cylinder rods ↔ boom | `lift_left_tip_closure`, `lift_right_tip_closure` |
| 17 | Boom ↔ front frame | `boom_joint` |
| 20 | Bell crank ↔ bucket link | `rod_joint` (passive) |
| 23 | Boom ↔ bell crank | `tilt_joint` (historical name) |

- The existing boom/crank/link/bucket graph was a valid closed four-bar topology, not four independent angle drives. It is retained. The old lumped-cylinder representation is replaced with explicit pinned barrels, physical prismatic rods and constrained tips.
- Cylinder and Z-bar loops are mandatory USD constraints in `closure.json`; bare URDF omits their final pins and is not a complete dynamics model. Barrel hinges, crank, boom, bucket and link receive no commanded rotary effort. Only cylinder sliders and wheel motors receive external efforts.

## Articulated-frame correction

- `rear_frame → articulation_joint → front_frame` remains one rotational DOF about +Z at the original hinge origin; ±38.5° is unchanged. Axles remain fixed to their corresponding frames, with independently rotating wheels. Rear-axle oscillation is not modeled.
- Estimated rear rails, crossmembers and upper/lower clevis supports connect to a separate rear hinge-housing body. Separate front sleeve/neck, low crossmembers, side members and boom-pivot towers provide actual contact geometry and load paths.
- Two estimated pin stubs and hollow bearing rings represent the manual's two-level hinge. Each ring exports as separate convex sectors so USD convex-hull conversion cannot fill the entire bore. Bearing clearances, casting thickness and exact pin dimensions are unmeasured estimates.
- Housing inertia and mass come from explicit subdivisions of the existing frame budget (rear 160 kg, front 110 kg); they are not decorative massless boxes or added residual mass. Compound tensors are recomputed from the configured primitives.
- Steering mounts are an estimated asymmetric longitudinal arrangement constrained by both manual end lengths and housing-clearance checks: rear X/Y ≈−0.1025/±0.3500 m; front X/Y ≈0.9394/±0.5000 m; Z≈−0.08 m relative to the hinge. These replace the earlier symmetric mounts, which collided with the housing near full articulation.
- Lift mounts retain their estimated planar positions and move inboard to Y=±0.24 m, with an explicit boom lift-pin crossmember. Towers and steering crossmembers leave actual cylinder corridors. These transverse dimensions and shapes are ESTIMATED, not brochure measurements.

## Geometry and identifiability

- Boom length ≈2.710 m; front-frame boom pivot ≈[0.810, 0.892] m in X/Z. Crank pivot ≈[1.701, 0.587] m in boom coordinates. Estimated crank lower pin ≈[−0.823, 0.180] m in the upper-pin-oriented crank frame; bucket link ≈0.744 m.
- The lower-pin estimate lies on a two-pose equal-link-length locus with a remaining free coordinate chosen for cylinder travel and closure. It is not uniquely identified. The rejected visible-D interpretation remains historical evidence, not an active manufacturer dimension.
- Visible boom/crank drawing fit residual: up to 8.4 mm; partly occluded boom-base residual: 30.9 mm. Bucket-anchor fit/held-out residuals: about 22/17 mm in the same drawing. These are drawing residuals, not true machine errors.
- Assumed sensitivity scales are 30 mm for selected digitized pivots and 200 mm for the unidentified lower pin. A universal 10–30 mm reconstruction-accuracy claim is not established.
- Bucket lip/upper-mouth outline is fitted to brochure workspace poses. Its maximum transient envelope differs from the brochure raised-rollback height; full CAD geometry, usable capacity and clean material cavity remain unverified.

## Cylinder coordinates and reduced hydraulics

- Service-manual reference dimensions: lift bore/rod/stroke 120/75/729 mm, pin lengths 1327–2056 mm; bucket 150/110/558 mm, 1354–1912 mm; steering 70/40/453 mm, 818–1271 mm. Installed-option applicability remains an uncertainty.
- Normalized commands specify bounded cylinder velocity references. `dv/dt = (v_cmd − v)/tau`, `dx/dt = v`; exact integration per control tick, reference stroke and outward-velocity saturation. Estimated tau S/L/B: 0.15/0.15/0.12 s; vmax: 0.08/0.14/0.16 m/s; Fmax per cylinder: 75/300/400 kN.
- A bounded PI-D force servo uses estimated 3 MN/m stiffness, 0.6 MN·s/m damping and 2 MN/(m·s) integral gain with anti-windup. The integral term stays inside the same force cap. This is not a pressure, valve, leakage, efficiency or engine-power-sharing model.
- Each barrel pivots at its configured frame attachment. Each rod slides along the cylinder axis and is constrained at its mechanical tip. Actual force is applied to the prismatic joint, with physical reactions through the barrel hinge and tip. No bucket-angle drive or hidden corrective joint torque is used.
- Prismatic coordinate zero is the **closed neutral assembly**, not full retraction. Physical extension = `q_slider + reference_length − manual_minimum`. This keeps the imported rest assembly closed and prevents the importer from deriving an incorrect second pin anchor. Neutral closure is checked before import.
- The bucket-cylinder base vector in the historical `linkage` section is relative to the boom-pivot origin; the explicit barrel mount includes that origin when expressed in the front frame. Mixing those frames is rejected by geometry/signature tests.
- The full rectangular boom/crank angle range is not a cylinder-valid, collision-free workspace. Tests use valid paths and check all five actual slider strokes and pin gaps.

## Deterministic signature and numerical validation

- `scripts/check_wa320_signature.py` writes `signature.csv` and `signature.png` locally: 51 lower-to-raised-rollback poses, boom/crank/bucket angles, tip position, absolute cylinder lengths, physical extensions and closure errors. It checks manual travel and rejects impossible geometry. This is an estimated-model signature, not machine validation.
- Independent URDF forward kinematics checks actual exported transforms and pin connections at 0°, ±20° and ±38.5°, plus fixed wheelbase/track/radius and one chassis yaw joint. An offset neutral cylinder length fails explicitly.
- Newton numerical profile uses explicit stop/closure time constants and impedance, with eight solver substeps. These enforce rigid mechanical pins; they do not impose a desired bucket pose. No body pose or velocity is overwritten after initialization/reset.
- The nominal Newton cycle and 19 native rigid-body cases pass. Recorded-state FK additionally audits every cylinder tip and stroke. Replay uses actual cylinder/link geometry; it no longer substitutes disconnected cylinder decorations or hides the physical bucket link.
- A separate 2000 kg ballast release uses explicitly stiff rigid contact (4 ms reference, priority 1) and checks ≤5 mm ballast overlap. Expected load/bucket contact is distinguished from vehicle self-collision.
- Rigid-pile reaction is tested; deformable penetration/material acquisition is not implemented. It must not be inferred from a contact-force or V-cycle pass.
