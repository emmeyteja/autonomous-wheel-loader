# WA320-7 drawing reconstruction

- Source: [Komatsu WA320-7 brochure, Form CJA00622-06, final page](https://kcsj.komatsu/-/media/kcsj/contact/download/pdf/construction_machine/wa320-7.pdf?hash=6FE96FE4CD51A0BC96B247F7157CA646&rev=b7bbb016972c43218296faf183be8782). Source checksum, rendering dimensions, manual pixel picks and assumptions live in `configs/geometry/wa320_7_brochure.yaml`. This is a drawing-based candidate, not CAD or measured machine geometry.
- The existing dynamic vehicle is not regenerated or promoted from these picks: the complete bucket/rod loop and cylinder mounts remain insufficiently identified. The reconstruction is an opt-in planar subassembly model and comparison against the active vehicle.

## Reproduce

    pip install -e '.[geometry-audit]'
    python scripts/reconstruct_wa320_brochure.py --video

- Optional: render your local brochure with `pdftoppm -f 16 -l 16 -scale-to 6000 -singlefile -png <brochure.pdf> <image-prefix>` and pass `--source-image <image-prefix>.png` to generate an annotated local crop. The required raster is 4240 × 6000 pixels; a different raster requires redigitization/rescaling.
- Outputs: `reconstruction.json`, `kinematics.npz`, a plot, optional MP4, and optional annotated crop under ignored `outputs/wa320/brochure_geometry/`. The source PDF, brochure artwork and generated videos are not redistributed in Git.
- Video is prescribed **kinematic replay**, not a dynamics simulation, measured trajectory, hydraulic model or complete PZ mechanism. Time and interpolation speed are arbitrary presentation choices. No forces are calculated.

## Coordinates and method

- Origin: front axle projected onto the ground; X forward (left on this side drawing), Z up. Scale is 3.030 m divided by the 419-pixel axle separation, or 7.232 mm/pixel. A common X/Z scale is assumed; illustration distortions are unknown.
- P: boom-to-bucket hinge; C: bellcrank pivot; E: tilt-cylinder pin. Two corresponding P/C pairs determine a best-fit proper planar rigid transform. Its fixed rotation centre gives B, the boom base. Average the back-transformed P/C coordinates to form a rigid boom. E determines crank orientation and average upper-pin radius in each pose.
- The separately picked B, the published 4.005 m raised hinge height and the nominal 0.695 m wheel/axle height are cross-checks within the **same brochure**, not independent physical validation. No published height or turning radius is fitted.
- D is a provisional rod-side crank pin in the raised drawing. T is a provisional fixed pin near the cylinder bases; its assignment to a particular cylinder is unconfirmed. Hidden lower-pose D and ambiguous bucket-side F are not invented as observations. Two candidate F circles are retained without choosing an assembly or claiming that they are the same attachment in different bucket poses.

## Candidate geometry and checks

- Fitted boom base: X=-0.705 m, Z=1.992 m in the front-axle/ground frame. Boom pin distance: **2.710 m**. Lower/raised boom elevations: -41.65° / 48.01°; these are observed illustrated positions, not verified mechanical stops.
- C lies at **[1.700, 0.589] m** in a boom frame with X along B→P. Current active geometry assumes zero transverse offset.
- C→E radius: 0.719 m. With provisional D, C→D is about 0.721 m and the included E-C-D angle is about **49.3°**, rather than the active straight-bar 180° assumption. This identifies a significant topology/geometry difference; D still needs confirmation.
- Largest fitted P/C/E discrepancy: 8.4 mm. Fitted B differs from the separately picked visible base by 30.9 mm. Raised P height differs from the published dimension by 3.2 mm; wheel height differs by 0.8 mm. These small residuals are drawing consistency, not millimetre-level vehicle accuracy.
- Perturbing visible picks independently by ±3 pixels and scale/ground picks by ±1 pixel yields 5th–95th percentile boom lengths of 2.669–2.756 m and C offsets of 0.572–0.609 m. These are sensitivity ranges under assumed errors, not confidence intervals. Pin misidentification, hidden geometry and drawing errors can be larger.
- Relative to this drawing, the active model's nominal base differs by about 0.470 m and lowered bucket hinge by 0.813 m, assuming the active equal hinge-to-axle split for frame conversion. That flags an approximation mismatch, not an independently measured real-machine error.
- Equal axle offsets and no-slip articulation predict an outer wheel-centre radius of 5.363 m and inner tire-envelope radius of 3.043 m, versus 5.380/3.060 m illustrated. Derived nominal tire width is 2.590−2.050=0.540 m under symmetric tire assumptions. R3060 denotes the illustrated inner tire envelope, **not** an inner wheel-centre constraint; all drawing dimensions are mm.

## Bucket audit update

- The subsequent [bucket registration and rod audit](WA320_BUCKET_GEOMETRY.md) prefers F hypothesis 1 but rejects promoting the original D/49.3° crank interpretation: it requires a 121 mm connecting-rod length change between the assumed corresponding poses. The values above remain a documented provisional hypothesis, not a resolved crank geometry. The bucket outline and full linkage have different validation status.

## Remaining identifiability limits

- The two F interpretations imply rod lengths of roughly 1.633 m or 1.456 m. These are competing attachment interpretations, not a measured tolerance range. Do not tune one until it appears to match or silently promote it to a physical model.
- A hypothetical T/E distance can be calculated, but the connection is unverified and the distance is not stroke without a retracted reference. Cylinder bore/rod diameter, stroke stops, pressures/flows, component mass properties and forces cannot be recovered from this outline.
- A verified F/D mapping and cylinder mounts, followed by closed-loop workspace/collision and simulator regression checks, are needed before regenerating the active vehicle. A full CAD model is not required if those measurements or clear linkage drawings become available.
