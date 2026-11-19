# WA320 reconstructed v1 linkage

- Source topology: PZ boom/front-frame, boom/bucket, boom/bellcrank, bellcrank/link, link/bucket, bucket cylinder/bellcrank, and paired lift cylinders/boom/frame. The URDF is a tree; `closure.json` and the supported USD conversion add the passive loop. Neither bucket nor link receives an independent drive.
- Original digitization and source checksums remain in `configs/geometry/`; reference methods remain in `docs/models/WA320_BROCHURE_GEOMETRY.md` and `WA320_BUCKET_GEOMETRY.md`.
- Active dimensions are centralized in `configs/vehicle/wa320.yaml`. The old v0 straight crank and zero-offset pivot remain reproducible from the archived v0 configuration and assets; they were not retuned.

## Estimated geometry and identifiability

- Boom length ≈2.710 m; front-frame pivot ≈[0.810, 0.892] m X/Z relative to the hinge. Crank pivot has an estimated nonzero transverse offset ≈[1.701, 0.587] m in boom coordinates.
- The earlier visible-D interpretation is rejected. V1 selects an explicitly estimated D on the two-pose equal-link-length locus, using the registered ground and raised rollback anchors. The remaining free coordinate is chosen to permit dump travel within the manual cylinder-length envelope and a continuous assembly branch. It is not a uniquely identified pin.
- D ≈[-0.823, 0.180] m relative to the upper-pin-oriented crank frame; connecting link ≈0.744 m. These large changes from the rejected D interpretation are assumptions, not manufacturer dimensions.
- Known visible boom/crank fit residuals are up to 8.4 mm; the partially occluded boom-base pick differs by 30.9 mm. Bucket-anchor registration residuals are about 22 mm in fitting and 17 mm in the held-out same drawing. These do not establish true pin errors.
- V1 stores assumed 30 mm sensitivity for selected digitized pivots and 200 mm for the unidentified D. A 10–30 mm true-geometry claim is **not established** for all pins. Unknown real errors are never reported as zero.
- Bucket lip and upper-mouth shape are constrained to the frozen dump reach/clearance and maximum-height targets. These checks are fitted workspace consistency, not independent validation of the brochure or bucket profile. The remaining outline is approximate, with open mouth, side walls and cutting-edge/floor plates. Heaped capacity is metadata, not a proven watertight 3.0 m³ volume.
- Some attachment proxies protrude into the empty bucket interior in the replay. Effective scoop volume and material/attachment collision clearance require checking in M2; the unloaded v1 tests do not validate filling.

## Cylinders and limits

- Verified against service-manual data, SEN06202-03 printed 01-11 / 01-16, serial 80001+: lift bore/rod/stroke 120/75/729 mm, pin lengths 1327–2056 mm; bucket 150/110/558 mm, lengths 1354–1912 mm; steering 70/40/453 mm, lengths 818–1271 mm.
- These nominal dimensions are accepted as v1 references for the selected 3.0 m³ configuration. Exact installed-option applicability remains an uncertainty; alternative bucket listings do not stop the approximation.
- Lift mounts are estimated from the two boom-end poses and the documented length envelopes. Steering mounts are an estimated symmetric arrangement constrained by end lengths at ±38.5°. Neither set of coordinates is manufacturer CAD.
- The full rectangular joint envelope is not a collision-free/cylinder-valid workspace. Scripted poses and the entire recorded path are checked for cylinder travel. Articulation and estimated assembly-branch joint envelopes remain explicit mechanical/numerical limits.

## First-order force actuation

- Existing control and simulation boundaries are retained. The waypoint reference generator feeds normalized `[drive, steering, lift, bucket]` commands through the actuator controller. The environment applies efforts; after reset it never writes body poses, velocities or joint positions to follow the path.
- Cylinder reference motion follows `dv/dt = (v_cmd - v)/tau`, `dx/dt = v`, integrated analytically per control step; normalized input sets bounded `v_cmd`. Here `x` is pin length and `x − minimum_length` is stroke. Saturation bounds reference stroke and outward velocity.
- Estimated tau [steer/lift/bucket]: 0.15/0.15/0.12 s; velocity limits 0.08/0.14/0.16 m/s; force limits 75/300/400 kN **per cylinder**. Lift uses two cylinders; steering uses equal/opposite left/right forces. These values are estimated, not identified from valve data.
- Actual length/rate comes from rigid linkage geometry. A bounded PI-D force servo transmits the reference into the mechanism, using estimated 3 MN/m stiffness and 0.6 MN·s/m damping, plus 2 MN/(m·s) integral gain with anti-windup. The integral term supports static loads inside the same force cap; it is not additional gravity compensation. Actual motion can lag or stall under load; no velocity clamp or teleport is applied to bodies.
- Forces map to articulation/boom/crank efforts through the cylinder-length Jacobian (virtual work), including bucket-cylinder reaction on the boom. Joint effort caps are conservative bounds derived from those force limits and maximum moment arms; legacy rotary-motor caps are retained only in v0. Passive bucket/link joints receive no drive or hidden corrective effort.
- Normalized drive uses a first-order estimated motor-speed reference and bounded wheel torque. This is a low-speed diagnostic drive model, not a calibrated HST powertrain. Cylinder bores do not imply valve dynamics, pressure traces or shared engine power.
- Numerical joint-stop/loop compliance is in the existing Newton profile. Active loop impedance is 0.9999: the older soft 0.9/0.95 closure could cross the estimated dump toggle gap (about 0.85 mm) and switch assembly branch. This is rigid-pin numerical enforcement, not a passive-joint drive. Force damping is part of the documented actuator approximation, not an undisclosed stabilization force. First-order state and masked resets are tested independently.

## Validation interpretation

- Algebraic closure is near floating-point precision; recorded dynamics use solver constraints and therefore have a separate nonzero loop gap.
- The current release Newton run reaches the V-cycle endpoints, curls/lifts, reverses raised, dumps, recurls at height before lowering, and returns to origin. The earlier PD-only run had a 0.086 mm gap but under-reached the raised hinge, so it does not complete the revised gates. This is numerical closure, not physical reconstruction accuracy.
- Recorded cylinder lengths stay inside manual ranges. Force and normalized-command limits are checked separately. The rigid ballast transfer is a separate native MuJoCo diagnostic of the exported geometry; no granular acquisition is claimed.

- Workspace checks use measured joint states at the documented raised-rollback and dump poses; targets alone do not count. The approximate bucket can have a larger transient top envelope between those poses. All-cycle maximum height is reported separately from the brochure pose comparison.
- Raise/dump sequencing aligns the bucket before raising and recurls to raised rollback before lowering. Conservative bellcrank slew is 0.04 rad/s; attachment feedback gates phase completion. These settings are diagnostic estimates, not an optimized loading policy.

- Final nominal Newton result: maximum loop gap 0.009 mm, return error 0.310 m; cylinder reference/force/travel and measured workspace screens pass. The maximum transient bucket-top envelope is 5.678 m, distinct from the brochure raised-rollback pose comparison (5.581 m).
