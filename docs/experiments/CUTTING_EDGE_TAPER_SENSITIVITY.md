# Cutting-edge taper and clearance sensitivity

## Controlled changes

- Extend [the existing entry fixture](CUTTING_EDGE_ENTRY.md); no new terrain framework or changes to reconstructed v1, original M2 controllers, material properties or frozen benchmarks.
- Three collision shapes: preserved `blunt`; `taper_5mm`; `taper_15mm`. Tapers remove material from the upper front of the first bottom plate over 0.150 m, ending at 0.005/0.015 m thickness. The lower face, full rear thickness (0.060 m), width and other colliders remain unchanged.
- These are **hypothetical sensitivity variants**, not measured WA320 bevels or teeth. Bucket mass, COM and inertia deliberately remain identical to isolate collision geometry. That is an ablation assumption, not a physically reconstructed lighter bucket.
- The thinner tip sits above the original lower face, below the original lip mid-plane. This changes the exposed tip surface/contact location as well as its slope. The reference lip pose and original cavity/mouth classifier remain fixed for comparisons.
- Same finite-force carriage, gains, limits, timestep, material, particle mass/radius, initial particle state and nominal command law within each comparison. Actual efforts/motion vary with contact and are recorded. This is feedback, not identical-effort replay.
- Two reference lip heights: 0.085 m and 0.075 m. Other references remain fixed. One particle and a three-particle transverse row; straight entry, nominal 0.40 m curl, nominal 0.95 m curl; three repetitions. Total: 36 small conditions / 108 runs, each with only one or three particles, not a pile-scale batch.
- The default API/CLI remains the original blunt 0.085 m fixture. Selecting an estimated variant is explicit. Geometry tests verify lower-face bounds, tip thickness, bevel transition, unchanged other colliders/materials and unchanged inertia.

## Free-particle results

- All 36 conditions pass the diagnostic gates: three identical recorded repetitions, finite forces/state, tracking within 10 mm/0.01 rad, no solver warnings, particle/floor overlap within 1 mm, balanced mass accounting and bucket/floor clearance at least 40 mm.
- **Zero mouth entry, acquisition or final bucket mass in every condition.** All particles remain near their ground height. Thinner tips and later curl do not produce loading in this free-particle fixture.
- At 0.085 m reference height, the three-particle first contact normal rises from **0.78° blunt** to **29.06° (5 mm)** or **23.56° (15 mm)** above horizontal.
- At 0.075 m, the respective elevations are **5.71°, 34.80° and 29.07°**. The particles still move forward; a more upward contact normal alone is insufficient here.
- Minimum bucket/floor clearance across the suite is **42.72 mm**. Accounting residuals and boundary losses are zero. Initial mass is 17.178466 kg or 51.535397 kg and remains unacquired; this ID sum is not a force-based payload measurement.
- All six original blunt 0.085 m traces are byte-identical to the earlier recorded qpos, contacts, accounting and carriage efforts. The added variants do not change that baseline.

## Back-pressure diagnostic

- Add an explicit static backstop with front face X=0.900 m; it uses copied floor contact properties. This is an artificial boundary test, **not a model of granular confinement**.
- Keep 0.085 m height and the same gains/limits/material. Straight entry only; cap nominal carriage advance at 0.700 m so the bucket does not reach the wall. Compare all shapes with one/three particles and three repeats (six conditions / eighteen runs).
- Record bucket/wall clearance independently: **at least 203.6 mm**. No direct bucket/wall contact explains the stall; the particles transmit the load.
- All cases again record zero mouth entry/acquisition and repeat identically. Single particles rise slightly (peak centre height 0.127–0.134 m), but do not enter. Row particles remain near the ground.
- **Every backstop condition fails the 10 mm/0.01 rad tracking gate.** Peak X following error is 11.6–34.2 mm. Five of six also fail the 40 mm clearance target; minimum actual bucket/floor clearance is 20.70 mm, still positive. Forces/state remain finite, no warnings, and accounting residuals remain zero.
- These are explicitly failed diagnostic cases. The evaluator returns nonzero; no gate was relaxed and no force/gain/material was increased to obtain a desired payload. Do not interpret the fixture's carriage following error as a validated WA320 hydraulic limitation.

## Conclusion and next experiment

- The original nearly horizontal blunt-edge contact is not the whole explanation: changing the contact direction does not establish entry. Freely moving particles escape forward; adding a rigid boundary produces a stalled finite-force carriage, not a valid scoop.
- **Do not promote either taper to the reconstructed loader or declare M2 complete.** No lift, reverse, payload retention, spillage, fuel, real-force accuracy or backend-selection claim follows from these tests.
- Next isolate the stalled contact's relative slip/rolling and bucket work, with contact-aware carriage stopping and clearance protection, before increasing the commanded advance or stiffness. Then use a small confined particle bed rather than treating a wall as soil. Keep material parameters fixed until those mechanisms are understood.
- Real tip thickness, bevel geometry, tooth geometry/spacing and a measured material/contact setup are still needed to validate an actual bucket model. The hypothetical dimensions here are not recommendations for a real machine.
- Limitations: coarse ~0.233 m-diameter spheres, no overburden in free tests, artificial wall in the pressure test, held bucket inertia, no chassis/linkage/hydraulics, and sampled contact forces at 240 Hz. Appearance and numerical repeatability are not physical accuracy.

## Reproduction and artifacts

```bash
for height in 0.085 0.075; do
  for variant in blunt taper_5mm taper_15mm; do
    for rep in 1 2 3; do
      PYTHONPATH=src .venv/bin/python scripts/run_edge_entry.py --variant "$variant" --attack-height "$height" --output "outputs/wa320/taper_entry/matched/h$height/$variant/repeat_$rep"
    done
    PYTHONPATH=src .venv/bin/python scripts/evaluate_edge_entry.py --root "outputs/wa320/taper_entry/matched/h$height/$variant"
  done
done
PYTHONPATH=src .venv/bin/python scripts/compare_edge_variants.py --root outputs/wa320/taper_entry/matched
for variant in blunt taper_5mm taper_15mm; do
  for rep in 1 2 3; do
    PYTHONPATH=src .venv/bin/python scripts/run_edge_entry.py --variant "$variant" --backstop --output "outputs/wa320/taper_entry/backstop_final/$variant/repeat_$rep"
  done
  PYTHONPATH=src .venv/bin/python scripts/evaluate_edge_entry.py --root "outputs/wa320/taper_entry/backstop_final/$variant"
  # This evaluation intentionally exits nonzero for its failed gates.
done
```

- `matched_comparison.json` checks common initial state/controller/source and records all metrics; `variant_comparison.png` shows measured particle heights/occupancy. Existing per-case contact/force plots and accounting are reused. Raw files remain local under ignored outputs.
- `scripts/render_edge_entry.py --case <run-folder>` accepts repeated cases, labels shape/height/boundary, shows contact-normal directions and displays failed gates. The recorded comparison includes free blunt, free taper at both heights, and the stalled backstop taper. It is explicitly an isolated fixture, not an M2 replay.
- Verification: 172 core tests plus fifteen integration checks pass (187 total); eight of those integration checks cover the entry fixtures, geometry/material preservation and backstop stall. Formatting/lint pass. Recorded failure cases and acceptance gates remain visible.
