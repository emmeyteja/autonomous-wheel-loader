# Obermayr blade-force reference

- [Obermayr et al. (2011)](https://doi.org/10.1016/j.jterra.2011.08.003), §§2.3–2.4 and 3.3. [Indexed paper text](https://www.academia.edu/34018170/Prediction_of_draft_forces_in_cohesionless_soil_with_the_Discrete_Element_Method) was checked beyond the publisher abstract. No source paper is redistributed.
- Calibration uses triaxial shear response at matched relative density and relevant stress. Internal friction is not the heap repose angle.
- Vertical blade: widths 50/100/200 mm, depths 20/40/60 mm, speed 10 mm/s, box plan 300 × 300 mm. Gravel bulk density: 1500 kg/m³. The reference-only YAML records the nine combinations and unresolved inputs.
- Force comparison uses the intercept H0 of a linear force-versus-travel fit, not simply mean or peak force (§3.3). Raw signals, fitting interval and uncertainty must be established before scoring our results.
- The model locks particle rotations; the authors acknowledge limitations for shear bands. Do not freeze our grains to obtain a matching force or slope.
- Source inconsistency: §2.4 reports experimental gravel radii 2–4 mm and numerical radii 1.5–3 mm, while §3.3.2 describes numerical grain enlargement. Verify original figures/model data before choosing a reproduction distribution.

## Use here

- Match preparation, blade geometry, boundaries, speed and force processing. Calibrate independently; validate across widths/depths. Missing fill height, thickness, tool friction and measured force records remain explicit nulls in `configs/benchmarks/obermayr_blade_reference.yaml`.
- A blade isolates soil/tool physics without a vehicle. Success would validate that test, not bucket filling, pile relaxation or loader dynamics.
- [The separate 2013 follow-up](https://publica-rest.fraunhofer.de/server/api/core/bitstreams/c7d65096-b1f3-4314-b727-0c720c54b79b/content), §3.1, compares locked rotations with rolling resistance in a larger sand experiment. Its apparatus and force averaging differ; do not combine them with the 2011 specification. Assess its rotating-particle model separately before adopting a compatible soil backend.
