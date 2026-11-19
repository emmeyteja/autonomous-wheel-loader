# Gravel heap reference and measurement protocol

- Selected material reference: [Klanfar et al. (2025)](https://doi.org/10.1038/s41597-025-04862-8) and [version 1 data](https://doi.org/10.6084/m9.figshare.28468526.v1). Public file metadata/checksums and apparatus values are in `configs/benchmarks/heap_scan_reference.yaml`; null fields remain unknown. The dataset record declares CC0, separately from the article's license. Raw archives stay in ignored outputs.
- `Gravel48` is 4–8 mm natural sieved aggregate. The described preparation mixes and pours 3.2 kg, levels the top in a 103 mm internal-diameter cylinder, and lifts vertically at 4 mm/s. Sample-specific fill height, full size/shape distribution, solid density and base friction still need measurement or recovery from associated records. Do not substitute Aoshima's different gravel values.
- The same dataset includes `QuartzSand0712`, dry quartz sand of 0.7–1.2 mm, so sand can use a separate physical reference. A gravel fit does not validate sand.
- A second gravel protocol is [Zhang et al. (2023)](https://doi.org/10.3390/pr11010268): irregular 10–25 mm gravel, baffle/drum calibration and a held-out cylinder test. Its validation cylinder is 200 mm diameter and 500 mm high, lifted at 10 mm/s. Missing fill details and their multi-sphere particle shapes prevent treating our spherical release as a reproduction.
- Keep Aoshima/Servin as the full-loader gravel benchmark. These material-only references avoid vehicle-shape differences; they do not validate bucket-force histories.

## Measurement-method check

- Use the dataset's independent `CAD35` surface to check the profile estimator before fitting material parameters. The expected 35° angle is used only for scoring, never to fit or move points. This verifies a geometric measurement method, not contact physics.
- Download the 5.6 MB CAD archive from the URL in the manifest and verify its MD5. Extract the specified member with a RAR-capable archive tool. SHA-256 identifies the extracted point file. No source data is bundled in this repository.
- The CAD cloud is level in its source XYZ axes. The declared Z datum is the minimum sampled CAD elevation; angle measurement needs equal axis scales but does not establish absolute length units. The estimator uses the mean XY sampling position and 10-source-unit radial bins. No automatic plane leveling, ground removal or angle forcing is applied. Recheck alignment/datum/masking separately for real scans.

    python scripts/check_heap_surface.py \
      --input outputs/heap_reference/20191201-CAD35-h0-m1-t1/CAD35.xyz \
      --delimiter comma --base-z -518.3507 --bin-width 10 \
      --expected-angle-deg 35 --tolerance-deg 0.5 \
      --source-doi 10.6084/m9.figshare.28468526.v1 \
      --output outputs/heap_reference/cad35_check.json

- Recorded result: four sector fits are 34.764–35.087°, maximum absolute error 0.24°, with R² above 0.9995. All pass the project's ±0.5° geometry-method screen. A cone does not test irregular-surface uncertainty, scanning error or material dynamics.

## Physical-validation eligibility

- Our 13 kg, 240 mm diameter, 250 mm high instantaneous column release is **not** this 3.2 kg slow-lift experiment. Its much coarser spheres and ordered initial fabric are also differences. Do not score the current material trial against these scans as matched physical accuracy.
- Before fitting: select specific heap repetitions; establish their reference frame/base, units, preparation, PSD/shape, density and moisture; implement the measured tube motion and boundary contacts. Check numerical stability and particle-resolution sensitivity in that apparatus.
- Calibrate on selected repetitions and validate on held-out repetitions/material conditions. Compare profiles and spatial surface errors as well as repose; document sampling/masking sensitivity and measurement uncertainty. Repose alone cannot identify all contact parameters.
- Scans provide final heap geometry, not bucket forces or dynamic collapse time series. Independent shear/tool experiments remain necessary before vehicle excavation validation. No calibrated material or production granular backend is enabled by the geometry-method check.
