# WA320-7 bucket outline and pin registration

- Uses the same [Komatsu brochure](https://kcsj.komatsu/-/media/kcsj/contact/download/pdf/construction_machine/wa320-7.pdf?hash=6FE96FE4CD51A0BC96B247F7157CA646&rev=b7bbb016972c43218296faf183be8782), raster and front-axle/ground frame as the [boom reconstruction](WA320_BROCHURE_GEOMETRY.md). Explicit bucket picks and assumptions are in `configs/geometry/wa320_7_bucket.yaml`.
- This adds an opt-in rigid bucket side outline, attachment registration and rejection test for the earlier provisional D pin. The active vehicle, collisions, masses and actuators are not changed. No complete passive curl linkage or force accuracy is claimed.

## Reproduce

    pip install -e '.[geometry-audit]'
    python scripts/reconstruct_wa320_bucket.py --video

- Optional `--source-image <4240x6000-brochure-raster.png>` produces a local registration overlay. Source preparation is described in the boom reconstruction.
- Ignored outputs: `bucket_reconstruction.json`, `bucket_kinematics.npz`, plotted/annotated images and `bucket_lift_curl_kinematic_replay.mp4`. Brochure artwork is not committed.
- Replay consumes recorded rigid profiles and hinge positions. It prescribes raise/rollback, dump and return over an arbitrary 16 s presentation. It does not integrate physics, enforce a rod loop, simulate payload or measure real timing. The dotted mouth closure and shaded polygon are schematic, not bucket plates or watertight collision geometry.

## Fit and cross-check

- P is the bucket hinge, F the bucket-side rod attachment, L the bolt-on cutting lip, M the upper mouth edge. One fixed local L/M/F set is fitted using ground and raised rollback views at assumed reference angles 0° and 66°. The raised dump view at -45° is excluded from fitting. Registration uses picked hinge positions; replay uses the fitted boom hinge, differing by up to about 6 mm. Angles and point identity are drawing interpretations, not verified stops.
- Fit local coordinates by averaging `R(-angle) × (feature−P)` across fitting poses. Trace the intermediate side-wall vertices from the ground view and align its L/M endpoints with the fitted features. Intermediate contour vertices are approximate and have no independent contour-error validation.
- Candidate local L=[1.445,-0.234] m, M=[1.009,1.143] m and F=[0.012,0.390] m. Nominal width 2.685 m and 3.0 m³ capacity are specification metadata; neither heaped capacity nor the physical shell is reconstructed from a 2D trace.
- Ground/rollback lip residual is 54 mm; mouth residual 28 mm; anchor residual 22 mm. In the excluded dump pose, residuals are 43/48/17 mm respectively. These are same-brochure checks, not independent real-machine accuracy.
- Predicted dump clearance is 2.819 m versus published 2.835 m (-16 mm); reach is 1.261 m versus 1.165 m (+96 mm). Reach subtracts nominal tire radius from lip X, to approximate the leading-tire reference, rather than using axle-centre reach. BOC endpoints, tire reference and outline interpretation remain uncertain. The reach mismatch is reported without adjusting the fit to erase it.
- Ground lip in the recorded fitted candidate is about 40 mm below the reference ground plane. This is a fit/preparation discrepancy, not simulated ground penetration or an accepted collision-free pose. The replay labels it; no ground-contact acceptance is claimed.

## Pin and rod conclusions

- F hypothesis 1 registers against the ground anchor with 44 mm discrepancy; hypothesis 2 gives 174 mm. The excluded dump anchor is consistent with hypothesis 1 to 17 mm after fitting.
- With independently assumed ±6-pixel feature/pin perturbations and ±1° pose-angle perturbations, 399/400 trials prefer hypothesis 1. This is a sensitivity result, not 99.75% confidence in true pin identity. Boom/base picks, hidden components and illustration errors are excluded.
- The old D assignment combined with these ground/rollback bucket poses requires rod lengths of **1.518 m and 1.639 m**. Their 121 mm spread fails the explicit 30 mm diagnostic screen. The earlier 49.3° triangular-crank interpretation therefore cannot be promoted as the one-rod linkage under these pose correspondences. Different pin or actuator-state correspondences could change this conclusion.
- [Aoshima and Servin, Fig. 2 and §5.2](https://doi.org/10.1007/s11044-024-10005-5) also shows the rod-side and tilt-side crank connections on opposite sides of the pivot. This qualitative topology supports rechecking D; the figure is not used as a dimensioned drawing or CAD substitute.
- Without a known D, equal rod length in two poses provides **one independent equation for its two planar coordinates**, leaving a family of D/rod-length pairs. The report records this constraint and the nearest algebraic solution only to demonstrate non-uniqueness. It is not an identified physical pin or a suggested replacement.
- One additional corresponding C/E/F pose, or a clear D/F linkage drawing/measurement, would help resolve that remaining freedom. Cylinder mounting identities, retracted lengths/strokes and lateral geometry remain separate missing inputs. Full passive closure, limits/collision and simulator regressions are required before active-asset promotion.
